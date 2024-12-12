import numpy as np
import pygame

import sys
import os
import pathlib
ROOT_DIR = str(pathlib.Path(__file__).parent.parent.parent)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from demonstration_collection.utils.replay_buffer import ReplayBuffer

def numpy_to_zarr(output='test.zarr', render_size=320, control_hz=10):
    # load numpy dataset
    dataset = np.load('demonstration_collection/dataset.npy', allow_pickle=True)
    
    # create replay buffer in read-write mode
    replay_buffer = ReplayBuffer.create_from_path(output, mode='a')

    clock = pygame.time.Clock()
    
    # episode-level while loop
    for _i in range(1):
        episode = list()
        # record in seed order, starting with 0
        seed = _i
        print(f'starting seed {seed}')
        
        # step-level loop
        for i in range(len(dataset)):

            img = dataset[i,0]
            state = dataset[i,1]
            action = dataset[i,1]

            # img = pygame.transform.scale(pygame.surfarray.make_surface(img), (96, 96))
            # img = pygame.surfarray.array3d(img)

            # dummy test data
            img = np.random.randint(0, 256, (240, 320, 3), dtype=np.uint8)
            # state = np.zeros(7)
            # action = np.zeros(7)
            
            data = {
                'img': img, # (96, 96, 3), uint8
                'state': np.float32(state), # (7)
                'action': np.float32(action), # (7)
            }
            episode.append(data)

        # save episode buffer to replay buffer (on disk)
        data_dict = dict()
        for key in episode[0].keys():
            data_dict[key] = np.stack(
                [x[key] for x in episode])
        replay_buffer.add_episode(data_dict, compressors='disk')
        print(f'saved seed {seed}')


if __name__ == "__main__":
    numpy_to_zarr()
