import os
import imageio
import natsort

def gen_gif(dirname):
    dirfiles = natsort.natsorted(os.listdir(dirname)) # 'frames/'
    images = [
        imageio.imread(f'{dirname}/{filename}')
        for filename in dirfiles
        if filename.lower().endswith(('.png', '.jpg', '.jpeg'))
    ]
    imageio.mimsave('_img_movie.gif', images)
    print(f'_img_movie.gif is saved in {os.getcwd()}')

