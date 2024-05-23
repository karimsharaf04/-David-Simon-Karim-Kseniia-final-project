import os
import torch
from torchvision import transforms
from torch.utils.data import DataLoader, Dataset
from PIL import Image
import torch.nn as nn
import torch.optim as optim

class CustomDataset(Dataset):
    def __init__(self, base_dirs, transform=None):
        self.transform = transform
        self.data = []
        self.labels = []

        # Label mapping for closed hand gestures
        closed_hand_labels = {'03_fist', '04_fist_moved'}

        # Traverse each directory in base_dirs
        for base_dir in base_dirs:
            for dir_name in os.listdir(base_dir):
                full_dir_path = os.path.join(base_dir, dir_name)
                if os.path.isdir(full_dir_path):
                    # Determine if the current directory contains closed hand images
                    label = 0 if dir_name in closed_hand_labels else 1
                    self.load_images_from_folder(full_dir_path, label)

    def load_images_from_folder(self, folder, label):
        for filename in os.listdir(folder):
            img_path = os.path.join(folder, filename)
            if os.path.isfile(img_path):
                self.data.append(img_path)
                self.labels.append(label)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        img_path = self.data[idx]
        image = Image.open(img_path).convert('RGB')
        label = self.labels[idx]
        if self.transform:
            image = self.transform(image)
        return image, label

# Define transformations
transformations = transforms.Compose([
    transforms.Resize((128, 128)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# List of base directories
base_dirs = [
    '/home/ksharaf/catkin_ws/src/-David-Simon-Karim-Kseniia-final-project/smaller_hands_data_set/00',
    '/home/ksharaf/catkin_ws/src/-David-Simon-Karim-Kseniia-final-project/smaller_hands_data_set/01',
    '/home/ksharaf/catkin_ws/src/-David-Simon-Karim-Kseniia-final-project/smaller_hands_data_set/02'
]

dataset = CustomDataset(base_dirs, transform=transformations)
train_loader = DataLoader(dataset, batch_size=32, shuffle=True)

# CNN model definition
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

def train_model(model, train_loader, epochs=10):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)

    for epoch in range(epochs):
        running_loss = 0.0
        for i, (images, labels) in enumerate(train_loader):
            images, labels = images.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
            print(f"Epoch {epoch+1}, Batch {i+1}/{len(train_loader)}, Loss: {loss.item():.4f}", end="\r")
        print(f'\nEpoch {epoch+1} complete, Average Loss: {running_loss/len(train_loader):.4f}')

    print("Training complete!")

def save_model(model, filename='model_closed_hand.pth'):
    torch.save(model.state_dict(), filename)
    print(f"Model saved to {filename}")

model = SimpleCNN()
train_model(model, train_loader)
save_model(model)
