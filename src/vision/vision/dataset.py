import os
import glob
import torch
import numpy as np
from PIL import Image
from torch.utils.data import Dataset
from torchvision import transforms

class SimSegmentationDataset(Dataset):
    """
    Dataset class for Simulation Segmentation data.
    Loads image-mask pairs from images/ and masks/ directories.
    """
    def __init__(self, data_dir, transform=None):
        self.data_dir = data_dir
        self.transform = transform
        
        self.images_path = os.path.join(data_dir, 'images')
        self.masks_path = os.path.join(data_dir, 'masks')
        
        self.image_files = sorted(glob.glob(os.path.join(self.images_path, '*.png')))
        self.mask_files = sorted(glob.glob(os.path.join(self.masks_path, '*.png')))
        
        assert len(self.image_files) == len(self.mask_files), \
            f"Number of images ({len(self.image_files)}) must match number of masks ({len(self.mask_files)})"

    def __len__(self):
        return len(self.image_files)

    def __getitem__(self, idx):
        img_path = self.image_files[idx]
        mask_path = self.mask_files[idx]
        
        image = Image.open(img_path).convert("RGB")
        mask = Image.open(mask_path).convert("L")  # Grayscale mask
        
        if self.transform:
            image = self.transform(image)
        else:
            image = transforms.ToTensor()(image)
            image = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])(image)
            
        # Convert mask to torch tensor (class indices)
        mask = torch.from_numpy(np.array(mask)).long()
        
        return image, mask
