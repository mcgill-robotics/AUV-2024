import numpy as np 
import opencv as cv


def process_video():
     pass

def load_images(path):
     pass

def save(final_image):
     pass


def stitching(final_image, cur_image):
     pass




if __name__ == "__main__":
     """
     1. final image should be a nested array where each element corresponds to an rgb value
     final_image = np.array() 
     
     2. we can either load a video and divide into images or just a set of images
     if images:
          load images()
     if video:
          save image every x seconds (this could be a parameter the user can set)

     3. run stitching function
     for image in images:
          stitching(final_image, image)
    
    4. save final image
    save(final_image)
     """     