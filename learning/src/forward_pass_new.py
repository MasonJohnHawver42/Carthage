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

class Neural_Network(nn.Module):
    def __init__(self):
        super().__init__()

        # Load weights for MobileNet_V3_Large_Weights
        model = models.mobilenet_v3_large(weights=models.MobileNet_V3_Large_Weights.IMAGENET1K_V1) 
        self.depth_backbone = model.features # output channels = 960
        #for param in self.backbone.parameters():
        #    param.requires_grad = False
        # self.depth_backbone_conv1d_1 = nn.Conv1d(in_channels=47040,out_channels=32,kernel_size=1)
        self.depth_backbone_conv1d_1 = nn.Conv1d(in_channels=47040,out_channels=32,kernel_size=3,stride=1,padding=1)

        # conv1d with kernel_size = 1 and stride = 1 is the same as a forward layer. But no reshape was needed
        self.states_backbone_forward_pass_1 = nn.Conv1d(in_channels=18, out_channels=64, kernel_size=1, stride=1)
        self.states_backbone_forward_pass_2 = nn.Conv1d(in_channels=64, out_channels=32, kernel_size=1, stride=1)
        self.states_backbone_forward_pass_3 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=1, stride=1)

        self.depth_backbone_relu = nn.LeakyReLU(negative_slope=0.5)

        # Get from shape [1,1,32] to [1,3,32] 
        self.backbone_conv1d_reshape = nn.Conv1d(in_channels=1,out_channels=3,kernel_size=1)

        # Output data are a set of 10 (x, y, z) coordinates and an associated trajectory cost => output_dim = 3*10 + 1
        self.concated_forward_pass_1 = nn.Conv1d(in_channels=64, out_channels=64, kernel_size=1, stride=1)
        self.concated_forward_pass_2 = nn.Conv1d(in_channels=64, out_channels=128, kernel_size=1, stride=1)
        self.concated_forward_pass_3 = nn.Conv1d(in_channels=128, out_channels=128, kernel_size=1, stride=1)
        self.concated_forward_pass_4 = nn.Conv1d(in_channels=128, out_channels=31, kernel_size=1, stride=1, padding=0)  # Assuming 'padding="same"' means no padding in PyTorch
        

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
        # Get images and run through pretrained Mobile V3 Net
        images = self.process_images(input[0]) # [batch_size, 3, 224, 224]
        backbone_depth = self.depth_backbone(images) # [batch_size, 960, 7, 7]

        # Reshape with convulutional 1D
        backbone_depth = backbone_depth.view(backbone_depth.shape[0],backbone_depth.shape[1],-1) # [batch_size,960,49]
        backbone_depth = backbone_depth.reshape(backbone_depth.shape[0], -1) # [batch_size, 960*49 = 47040]
        backbone_depth = backbone_depth.unsqueeze(-1) # [batch_size, 47040, 1]
        backbone_depth = self.depth_backbone_conv1d_1(backbone_depth) # [batch_size, 32, 1]
        backbone_depth = self.depth_backbone_relu(backbone_depth)
        backbone_depth = backbone_depth.permute(0, 2, 1) # [batch_size, 1, 32]
        backbone_depth = self.backbone_conv1d_reshape(backbone_depth) # [batch_size, 3, 32]
        backbone_depth = self.depth_backbone_relu(backbone_depth)

        # Get imu from input and run through forward layers
        backbone_states = input[1] #  [batch_size, 18]
        backbone_states = backbone_states.unsqueeze(-1)  # [batch_size, 18, 1]
        backbone_states = self.states_backbone_forward_pass_1(backbone_states)
        backbone_states = self.depth_backbone_relu(backbone_states)
        backbone_states = self.states_backbone_forward_pass_2(backbone_states)
        backbone_states = self.depth_backbone_relu(backbone_states)
        backbone_states = self.states_backbone_forward_pass_3(backbone_states)
        backbone_states = self.depth_backbone_relu(backbone_states)
        backbone_states = self.states_backbone_forward_pass_3(backbone_states) # [batch_size, 32, 1]
        backbone_states = backbone_states.permute(0,2,1) # [batch_size, 1, 32]
        backbone_states = self.backbone_conv1d_reshape(backbone_states) # [batch_size,3,32]

        # Combine depth backbone and state backbone
        concated_backbones = torch.cat((backbone_depth, backbone_states), dim=2) # output_dimension = [batch_size,3,64] where likely modes = 3
        concated_backbones = concated_backbones.permute(0,2,1) # [batch_size, 64, 3]

        # Run through [64,64,128,128] forward layer
        concated_backbones = self.concated_forward_pass_1(concated_backbones)
        concated_backbones = self.concated_forward_pass_2(concated_backbones)
        concated_backbones = self.concated_forward_pass_3(concated_backbones)
        trajectories = self.concated_forward_pass_4(concated_backbones)

        return trajectories