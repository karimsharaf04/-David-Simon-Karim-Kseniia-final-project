import torch
import torchvision.transforms as transforms
from torch.utils.data import DataLoader, Dataset
import torch.nn as nn
from PIL import Image
import os

# Define the CNN model architecture
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(32, 64, 3, padding=1)
        self.fc1 = nn.Linear(64 * 32 * 32, 256)
        self.fc2 = nn.Linear(256, 2)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = torch.flatten(x, 1)
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x

# Custom dataset class
class CustomDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform
        self.images = [os.path.join(root_dir, file) for file in os.listdir(root_dir) if file.endswith(('png', 'jpg', 'jpeg'))]
        print(f"Loaded {len(self.images)} images from {root_dir}")

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_path = self.images[idx]
        image = Image.open(img_path).convert('RGB')
        if self.transform:
            image = self.transform(image)
        return image, 1  # Assuming all test images are 'other' gestures (non-closed)

# Define transformations
transformations = transforms.Compose([
    transforms.Resize((128, 128)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# Set up the dataset
dataset = CustomDataset(root_dir='/home/ksharaf/catkin_ws/src/-David-Simon-Karim-Kseniia-final-project/smaller_hands_data_set/02/04_fist_moved', transform=transformations)
loader = DataLoader(dataset, batch_size=32, shuffle=False)

# Load the newly trained model
model = SimpleCNN()
model.load_state_dict(torch.load('new_closed_hand.pth'))  # Ensure this is the correct path
model.eval()
print("New model loaded and set to evaluation mode.")

# Evaluate the model and print predictions
predictions = []

with torch.no_grad():
    for images, _ in loader:  # Ignore labels since they are dummies
        outputs = model(images)
        _, predicted = torch.max(outputs, 1)
        predictions.extend(predicted.tolist())

# Analyze predictions
print("Prediction distribution:", {x: predictions.count(x) for x in set(predictions)})
