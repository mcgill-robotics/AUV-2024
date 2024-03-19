import numpy as np 
import cv2


def process_video(path):
     pass

def load_images(path):
     pass

def load_positions(path):
     pass

def save(final_image):
     pass


def stitching(final_image, cur_image, cur_position):
     pass




if __name__ == "__main__":
     """     
     1. final image should be a nested array where each element corresponds to an rgb value
     final_image = []
     
     2. we can either load a video and divide into images or just a set of images
     store images as a list
     path_images = ???
     images = load_images(path_images)
     OR
     images = process_video(path_images)

     3. load position and orientation for each image:
     path_positions = ???
     positions = load_positions(path_positions)

     if len(positions) != len(images):
          raise error

     4. run stitching function
     for i in range(len(images)):
          stitching(final_image, images[i], positions[i])
    
     5. save final image
     save(final_image)
     """     