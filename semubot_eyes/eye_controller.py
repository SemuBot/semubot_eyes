#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import pygame

#variables for the filter
arr = []
n = 0

#pygame inits
pygame.init()
pygame.display.set_caption("Semuscreen")


#all imported images
#draw pupils on the x-axis using an image with a transparent background
lid_image = pygame.image.load("src/semubot_eyes/images/lidst.png") # Use an image with a transparent background!
outline_image = pygame.image.load("src/semubot_eyes/images/outline.png") # Use an image with a transparent background!
pupil_image = pygame.image.load("src/semubot_eyes/images/eyebb.png")  # Use an image with a transparent background!
mouth_image = pygame.image.load("src/semubot_eyes/images/mouth.png") # Use an image with a transparent background!

class RespeakerSubscriber(Node):
    
    def __init__(self):
        super().__init__('eye_controller')
        
        #eye values/parameters
        self.width, self.height = 1920, 1080
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)

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

        mouth_image_scaled = pygame.transform.scale(mouth_image, (self.lid_width/2.5, self.lid_height/5.5))

        self.screen.blit(lid_image_scaled, (-60, 100))
        self.screen.blit(outline_image_scaled, (-60, 100))
        self.screen.blit(mouth_image_scaled, (715, 1020))
        
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
                elif event.type == pygame.VIDEORESIZE:
                    #handle window resize event
                    eye_controller.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)

            #process ROS messages
            rclpy.spin_once(eye_controller, timeout_sec=0)

            #update display
            eye_controller.screen.fill((255, 255, 255))
            eye_controller.draw_eyes()
            pygame.display.flip()
            pygame.time.delay(10)

    except KeyboardInterrupt:
        pass

    finally:
        eye_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
