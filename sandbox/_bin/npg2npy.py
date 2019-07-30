### ADD DOCUMENTATION
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt

filename = 'image-test'

img = Image.open( filename + '.png' )
data = np.array( img, dtype='uint8' )

np.save( filename + '.npy', data)

# visually testing our output
img_array = np.load(filename + '.npy')
plt.imshow(img_array) 