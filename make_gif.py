from PIL import Image
import os

# Set the path for the directory containing images
dir_path = 'gif_images/'

# Get a list of all image files in the directory
files = os.listdir(dir_path)
image_files = [os.path.join(dir_path, f) for f in files if f.endswith('.png')]
# print(image_files)

# Sort the image files in numerical order
image_files.sort(key=lambda x: int(os.path.splitext(x[11:])[0]))

# Create an empty list to store the image objects
images = []

# Loop through the image files and append each image to the list
for filename in image_files:
    images.append(Image.open(filename))

# Save the images as an animated GIF
gif_path = 'world_1.gif'
images[0].save(gif_path, save_all=True, append_images=images[1:], duration=100, loop=0)
