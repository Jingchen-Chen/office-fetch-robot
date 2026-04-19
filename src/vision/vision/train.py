import argparse
import os
import json
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from .unet_model import UNet
from .dataset import SimSegmentationDataset
from .metrics import compute_miou, compute_pixel_accuracy
import numpy as np

class DiceLoss(nn.Module):
    """
    Dice loss implementation for semantic segmentation
    """
    def __init__(self, weight=None, size_average=True):
        super(DiceLoss, self).__init__()

    def forward(self, inputs, targets, smooth=1):
        # inputs: logits, targets: class indices
        inputs = torch.softmax(inputs, dim=1)
        
        # one-hot encoding for targets
        num_classes = inputs.shape[1]
        targets_one_hot = torch.nn.functional.one_hot(targets, num_classes).permute(0, 3, 1, 2).float()
        
        # flatten label and prediction tensors
        inputs = inputs.contiguous().view(-1)
        targets_one_hot = targets_one_hot.contiguous().view(-1)
        
        intersection = (inputs * targets_one_hot).sum()                            
        dice = (2.*intersection + smooth)/(inputs.sum() + targets_one_hot.sum() + smooth)  
        
        return 1 - dice

def train():
    """
    Training script usage:
    python3 -m vision.train --data_dir /path/to/data --epochs 50 --batch_size 8 --lr 1e-3
    """
    parser = argparse.ArgumentParser(description='Train U-Net for Semantic Segmentation')
    parser.add_argument('--data_dir', type=str, required=True, help='Path to dataset directory')
    parser.add_argument('--epochs', type=int, default=50, help='Number of epochs')
    parser.add_argument('--batch_size', type=int, default=8, help='Batch size')
    parser.add_argument('--lr', type=float, default=1e-3, help='Learning rate')
    parser.add_argument('--num_classes', type=int, default=2, help='Number of classes')
    parser.add_argument('--save_dir', type=str, default='checkpoints', help='Directory to save models')
    parser.add_argument('--use_dice', action='store_true', help='Use Dice loss instead of CrossEntropy')
    
    args = parser.parse_args()
    
    # Create save directory
    os.makedirs(args.save_dir, exist_ok=True)
    
    # Device configuration
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Dataset and DataLoader
    full_dataset = SimSegmentationDataset(args.data_dir)
    n_val = int(len(full_dataset) * 0.2)
    n_train = len(full_dataset) - n_val
    train_ds, val_ds = random_split(full_dataset, [n_train, n_val])
    
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=2)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=2)
    
    # Model
    model = UNet(in_channels=3, num_classes=args.num_classes).to(device)
    
    # Loss and Optimizer
    if args.use_dice:
        criterion = DiceLoss()
    else:
        criterion = nn.CrossEntropyLoss()
        
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.1)
    
    # Training Loop
    best_miou = 0.0
    history = {
        'train_loss': [],
        'val_loss': [],
        'val_miou': [],
        'val_acc': []
    }
    
    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for images, masks in train_loader:
            images = images.to(device)
            masks = masks.to(device)
            
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, masks)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            
        train_loss /= len(train_loader)
        
        # Validation
        model.eval()
        val_loss = 0.0
        val_miou = 0.0
        val_acc = 0.0
        
        with torch.no_grad():
            for images, masks in val_loader:
                images = images.to(device)
                masks = masks.to(device)
                
                outputs = model(images)
                loss = criterion(outputs, masks)
                val_loss += loss.item()
                
                miou = compute_miou(outputs, masks, args.num_classes)
                acc = compute_pixel_accuracy(outputs, masks)
                
                val_miou += miou
                val_acc += acc
                
        val_loss /= len(val_loader)
        val_miou /= len(val_loader)
        val_acc /= len(val_loader)
        
        scheduler.step()
        
        # Log progress
        print(f"Epoch [{epoch}/{args.epochs}] | Train Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f} | Val mIoU: {val_miou:.4f} | Val Acc: {val_acc:.4f}")
        
        # Update history
        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        history['val_miou'].append(val_miou)
        history['val_acc'].append(val_acc)
        
        # Save best model
        if val_miou > best_miou:
            best_miou = val_miou
            torch.save(model.state_dict(), os.path.join(args.save_dir, 'best_model.pth'))
            print(f"--> Saved new best model with mIoU: {best_miou:.4f}")
            
        # Save last model
        torch.save(model.state_dict(), os.path.join(args.save_dir, 'last_model.pth'))
        
        # Save metrics
        with open(os.path.join(args.save_dir, 'metrics.json'), 'w') as f:
            json.dump(history, f)

if __name__ == '__main__':
    train()
