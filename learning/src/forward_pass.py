import torch
import numpy as np
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import torchvision.models as models
import cv2
from PIL import Image

# br in the trajectory refers to "body rate" btw
# Input to the network: depth image, rgb image(optional, don't think we are gonna include this), imu object (contains position, velocity, rotation matrix, goal direction)
# imu object for each traj is an 1-D array of length 3(position) + 9(attitude as rot mat) + 3(velocity) + 3(goal direction) + 1 (rollout id) = 19 assuming body rates of the drone aren't used; if rollout_id ain't needed, this will be 18

# batch_input: 
    # I assume input is comprised of two things
        # first thing: tensor containing image data of shape [batch_size, H, W, channels(1)]; [depth images have 1 channel]
        # second thing: tensor containing imu_obj data of shape [batch_size, len(imu_obj) = 18/19]
        # i'th thing can be accessed by input[i] 
    # In case we are not dealing with batches, make sure the first dimension is 1 so it can be treated the same way

class Plan_Network(nn.Module):
    def __init__(self):
        super().__init__()
        model = models.mobilenet_v3_large(weights=models.MobileNet_V3_Large_Weights.IMAGENET1K_V1) 
        self.backbone = model.features # output channels = 960
        #for param in self.backbone.parameters():
        #    param.requires_grad = False
        self.resize_op = nn.Conv1d( in_channels=960, out_channels=128, kernel_size=1, stride=1, padding=0, dilation=1, bias=True) # input channel = 960
        self.modes = 3
        self.img_mergenet = nn.ModuleList([ #input channel = 49*128 = 6272
            nn.Conv1d(in_channels=6272, out_channels=128, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Conv1d(in_channels=128, out_channels=64, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Conv1d(in_channels=64, out_channels=64, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Conv1d(in_channels=64, out_channels=32, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=1e-2),
            nn.Conv1d(in_channels=32, out_channels=32, kernel_size=2, stride=1, padding='same')
        ])
        self.resize_op_2 = nn.Conv1d(in_channels=1, out_channels=self.modes, kernel_size=3, stride=1, padding=1, bias=True)
        
        self.states_conv = nn.ModuleList([
            nn.Conv1d(in_channels=18, out_channels=64, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=0.5),
            nn.Conv1d(in_channels=64, out_channels=32, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=0.5),
            nn.Conv1d(in_channels=32, out_channels=32, kernel_size=2, stride=1, padding='same'),
            nn.LeakyReLU(negative_slope=0.5),
            nn.Conv1d(in_channels=32, out_channels=32, kernel_size=2, stride=1, padding='same')
        ])
        self.resize_op_3 = nn.Conv1d(in_channels=1, out_channels=self.modes, kernel_size=3, stride=1, padding=1, bias=True)
        # assuming output data are a set of 10 (x, y, z) coordinates and an associated trajectory cost => output_dim = 3*10 + 1
        output_dim = 31
        self.plan_module = nn.ModuleList([
            nn.Conv1d(in_channels=64, out_channels=64, kernel_size=1, stride=1),
            nn.LeakyReLU(negative_slope=0.5),
            nn.Conv1d(in_channels=64, out_channels=128, kernel_size=1, stride=1),
            nn.LeakyReLU(negative_slope=0.5),
            nn.Conv1d(in_channels=128, out_channels=128, kernel_size=1, stride=1),
            nn.LeakyReLU(negative_slope=0.5),
            nn.Conv1d(in_channels=128, out_channels=output_dim, kernel_size=1, stride=1, padding=0)  # Assuming 'padding="same"' means no padding in PyTorch
        ])
        

    def process_images(self, img_tensor):
        # assuming img_tensor has shape [batch_size, H, W, channel]
        batch_size = img_tensor.shape[0]
        processed_images = []
        transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Grayscale(num_output_channels=3),
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        single_image_data = img_tensor[0, :, :, 0]
        processed_images = transform(single_image_data).unsqueeze(0)
        for i in range(1, batch_size):
            single_image_data = img_tensor[i, :, :, 0]
            torch.cat((processed_images, transform(single_image_data).unsqueeze(0)), axis=0)
        
        processed_images = torch.tensor(processed_images)
        return processed_images # shape = [batch_size, 3, 224, 224]

    def forward(self, input):
        images = self.process_images(input[0]) # shape = [batch_size, 3, 224, 224]
        imu = input[1] # shape = [batch_size, 18/19]
        # process image into image embeddings:

        # Step 1: feature extraction using bacbone model
        x = self.backbone(images) # output_dimension = [batch_size, 960, 7, 7]
        x = x.reshape(x.shape[0], x.shape[1], -1) # output_dimension = [batch_size, 960, 49]
        x = self.resize_op(x) # output_dimension = [batch_size, 128, 49]
        x = x.reshape(x.shape[0], -1) # output_dimension = [batch_size, 128*49 = 6272]
        
        # Step 2: Create Img Embedding
        x = x.unsqueeze(-1) # output_dimension = [batch_size, 6272, 1]
        for f in self.img_mergenet:
            x = f(x) # output_dimension = [batch_size, 32, 1]
        
        x = x.permute(0, 2, 1) # output_dimension = [batch_size, 1, 32]
        x = self.resize_op_2(x) # output_dimension = [batch_size, modes, 32] where modes = 3 for now
        x = x.permute(0, 2, 1) # output_dimension = [batch_size, 32, modes]
        image_embedding = x # [batch_size, 32, modes]

        # Process imu_obj into imu embedding
        y = imu.unsqueeze(-1)  # output_dimension = [batch_size, 18, 1]
        for f in self.states_conv:
            y = f(y)  # output_dimension = [batch_size, 32, 1]
        y = y.permute(0, 2, 1) # output_dimension = [batch_size, 1, 32]
        imu_embedding = self.resize_op_3(y) # output_dimension = [batch_size, modes, 32] where likely modes = 3
        imu_embedding = imu_embedding.permute(0, 2, 1) # output_dimension = [batch_size, 32, modes]
        # combine embeddings
        total_embedding = torch.cat((image_embedding, imu_embedding), dim=1) # output_dimension = [batch_size, 64, modes] where likely modes = 3
        # Output
        output = total_embedding
        for f in self.plan_module:
            output = f(output) # output_dimension = [batch_size, 3*10 + 1 , modes] = [batch_size, 31, mode]

        return output