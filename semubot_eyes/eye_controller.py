#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import pygame
import numpy as np 
import scipy.io.wavfile as wav 
import subprocess
import threading
import os

#variables for the filter
arr = []
n = 0

#pygame inits
pygame.init()
pygame.display.set_caption("Semuscreen")
pygame.display.set_mode((0, 0), pygame.FULLSCREEN | pygame.NOFRAME)

# Get paths to the ROS packages
eyes_package_name = 'semubot_eyes'  
eyes_package_path = get_package_share_directory(eyes_package_name)

img_dir_path = os.path.join(eyes_package_path, "images")

#all imported images
#draw pupils on the x-axis using an image with a transparent background
lid_image = pygame.image.load(os.path.join(img_dir_path, "lidst.png")) # Use an image with a transparent background!
outline_image = pygame.image.load(os.path.join(img_dir_path, "outline.png")) # Use an image with a transparent background!
pupil_image = pygame.image.load(os.path.join(img_dir_path, "eyebb.png"))  # Use an image with a transparent background!
mouth_image = pygame.image.load(os.path.join(img_dir_path, "mouth3.png")) # Use an image with a transparent background!
mouth_image2 = pygame.image.load(os.path.join(img_dir_path, "mouth4.png")) # Use an image with a transparent background!

wav_file_path = os.path.join(eyes_package_path, 'test_speech.wav')

sample_rate, adata = wav.read(wav_file_path)

class RespeakerSubscriber(Node):
    
    def __init__(self):
        super().__init__('eye_controller')
        
        #eye values/parameters
        self.width, self.height = 1920, 1080
        #self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN | pygame.NOFRAME)

        self.eye_radius = 250
        self.pupil_radius = 80
        self.eye_distance = 650
        self.eye_y = self.height // 2
        self.lid_scale_factor = 1 * self.eye_distance / 2388
        self.lid_width = int(2388 * self.lid_scale_factor) * 2.135
        self.lid_height = int(1668 * self.lid_scale_factor) * 2

        #subscribe to topics published by the publisher node
        self.subscription_doa_raw = self.create_subscription(Int32, 'doa_raw', self.doa_raw_callback, 10)
        self.subscription_doa = self.create_subscription(PoseStamped, 'doa', self.doa_callback, 10)

        self.eye_x = self.width // 2 - self.eye_distance // 2 - 55  # Adjust x eye positions here, default -> 750

        self.angle = 0
        self.direction = 0
        

        self.mouth_image = mouth_image
        self.mouth_image2 = mouth_image2
        self.current_mouth_image = self.mouth_image
        self.speech_detected = False
        self.frame_count = 0

    def doa_raw_callback(self, msg):
        #callback function for 'doa_raw' topic
        self.get_logger().info('DOA raw: "%s"' % msg.data)

        self.angle = msg.data 
        self.direction = self.angle

    def draw_eyes(self):
        #calculate pupil x-coordinate based on DOA (left to right movement)
        left_eye_x = self.width // 2 - self.eye_distance // 2 - 270 #adjust x eye positions here, default -> 750
        right_eye_x = self.width // 2 + self.eye_distance // 2 + 240 #default -> 2650

        lid_image_scaled = pygame.transform.scale(lid_image, (self.lid_width*1.5, self.lid_height*1.5))

        outline_image_scaled = pygame.transform.scale(outline_image, (self.lid_width*1.5, self.lid_height*1.5))
        
        mouth_image_scaled = pygame.transform.scale(mouth_image, (self.lid_width/1.5, self.lid_height/2.5))
        # mouth_image_scaled = pygame.transform.scale(mouth_image, (self.lid_width, self.lid_height))

        self.screen.blit(lid_image_scaled, (-60, 100))
        self.screen.blit(outline_image_scaled, (-60, 100))
        if self.is_wav_playing(wav_file_path) == True:
            if self.frame_count % 5 == 0:
                self.current_mouth_image = self.mouth_image2 
                self.screen.blit(self.current_mouth_image, (620, 850))
                # self.frame_count = 0
            else:
                self.screen.blit(mouth_image_scaled, (515, 920))
            self.frame_count += 1
                
        else:
            
            self.screen.blit(mouth_image_scaled, (515, 920))

        
        if abs(self.direction) > 120: #lasy control of the borders
            self.direction = self.direction * 0.5

        pupil_x_left = left_eye_x + self.direction 
        pupil_x_right = right_eye_x + self.direction 
        
        #adjust the y-coordinate to center the pupils within the lids
        pupil_y = self.eye_y - self.pupil_radius * 2 - (-30) #adjust y eye position here
        pupil_img = pygame.transform.scale(pupil_image, (self.pupil_radius * 5, self.pupil_radius * 5))
        self.screen.blit(pupil_img, (pupil_x_left - self.pupil_radius * 2, pupil_y))
        self.screen.blit(pupil_img, (pupil_x_right - self.pupil_radius * 2, pupil_y))

    def doa_callback(self, msg):
        self.get_logger().info('DOA: "%s"' % msg.pose)

    def is_wav_playing(self, wav_file):
        try:
            # Get all running processes
            result = subprocess.run(['pgrep', '-af', 'play.*test_speech.wav'], stdout=subprocess.PIPE) # TODO: remove hardcoded .wav name
            output = result.stdout.decode()

            # Debugging: print the output of pgrep
            print("Current 'play' processes:", output)
            print("wav_file:", wav_file)

            # Check if the WAV file is in the output
            if len(output) > 0:
                # print(f"The file {wav_file} is currently playing.")
                return True
        except Exception as e:
            print(f"Error checking for playing process: {e}")

        # print(f"The file {wav_file} is not currently playing.")
        return False


            
    def update_mouth(self):
        
        if self.is_wav_playing(wav_file_path) == True:
            # # Analyze audio data for speech detection
            # if self.frame_count < len(adata):
            #     frame = adata[self.frame_count]
            #     # Check if the amplitude exceeds a threshold (e.g., 500)
            #     if np.abs(frame).max() > 500:
            #         self.speech_detected = True
            #     else:
            #         self.speech_detected = False
                
                # Switch mouth image if speech is detected
            # if self.speech_detected == True:
            # if self.frame_count % 400 == 0:  # Change image every 20 frames
                self.current_mouth_image = self.mouth_image2 
                self.screen.blit(self.current_mouth_image, (620, 850))
            
            # self.frame_count += 1
            # self.get_logger().info('Detect: "%s"' % self.speech_detected)
    


def main(args=None):
    rclpy.init(args=args)
    eye_controller = RespeakerSubscriber()

    try:
        while rclpy.ok():
            #process Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
                #elif event.type == pygame.VIDEORESIZE:
                    #handle window resize event
                #    eye_controller.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)

            #process ROS messages
            rclpy.spin_once(eye_controller, timeout_sec=0)

            #update display
            eye_controller.screen.fill((255, 255, 255))
            eye_controller.draw_eyes()
            # eye_controller.update_mouth()
            pygame.display.flip()
            pygame.time.delay(10)

    except KeyboardInterrupt:
        pass

    finally:
        eye_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
