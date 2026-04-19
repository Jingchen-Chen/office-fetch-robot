import torch
import numpy as np

def compute_iou(pred, target, num_classes):
    """
    Computes per-class intersection over union (IoU)
    """
    ious = []
    # If pred is logits, take argmax
    if pred.ndim == 4:
        pred = torch.argmax(pred, dim=1)
        
    for cls in range(num_classes):
        pred_cls = (pred == cls)
        target_cls = (target == cls)
        
        intersection = (pred_cls & target_cls).float().sum()
        union = (pred_cls | target_cls).float().sum()
        
        if union == 0:
            # If no ground truth and no prediction, IoU is 1.0 (or NaN/excluded)
            # In training, we often exclude classes not present in ground truth
            ious.append(float('nan'))
        else:
            ious.append((intersection / union).item())
            
    return np.array(ious)

def compute_miou(pred, target, num_classes):
    """
    Computes mean intersection over union (mIoU)
    """
    ious = compute_iou(pred, target, num_classes)
    # Ignore NaN values in mIoU calculation
    valid_ious = ious[~np.isnan(ious)]
    if len(valid_ious) == 0:
        return 0.0
    return np.mean(valid_ious)

def compute_dice(pred, target, num_classes):
    """
    Computes per-class Dice coefficient
    """
    dices = []
    if pred.ndim == 4:
        pred = torch.argmax(pred, dim=1)
        
    for cls in range(num_classes):
        pred_cls = (pred == cls)
        target_cls = (target == cls)
        
        intersection = (pred_cls & target_cls).float().sum()
        total_pixels = pred_cls.float().sum() + target_cls.float().sum()
        
        if total_pixels == 0:
            dices.append(float('nan'))
        else:
            dices.append((2. * intersection / total_pixels).item())
            
    return np.array(dices)

def compute_pixel_accuracy(pred, target):
    """
    Computes overall pixel accuracy
    """
    if pred.ndim == 4:
        pred = torch.argmax(pred, dim=1)
    
    correct = (pred == target).float().sum()
    total = target.numel()
    
    return (correct / total).item()
