#!/usr/bin/env python3

import torch
import torchvision.models as models
import torch.nn as nn
import os

def prepare_and_save_model():
    print("Downloading pretrained MobileNetV2...")
    # Load a pretrained MobileNetV2 model
    weights = models.MobileNet_V2_Weights.DEFAULT
    model = models.mobilenet_v2(weights=weights)

    # By default, MobileNetV2 classifies 1000 different objects.
    # We only care about 1 thing: Grasp Success Probability [0 to 1].
    # So, we replace the final layer with a single output + Sigmoid.
    model.classifier[1] = nn.Sequential(
        nn.Linear(model.last_channel, 1),
        nn.Sigmoid()
    )

    # Ensure the models directory exists
    os.makedirs('models', exist_ok=True)

    # Save the model weights (PyTorch uses .pth instead of Keras' .h5)
    save_path = 'models/grasp_quality_model.pth'
    torch.save(model.state_dict(), save_path)
    
    print(f"Success! Model weights saved to {save_path}")

if __name__ == '__main__':
    prepare_and_save_model()