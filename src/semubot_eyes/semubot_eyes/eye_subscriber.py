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
lid_image = pygame.image.load("src/semubot_eyes/lidst.png") # Use an image with a transparent background!
outline_image = pygame.image.load("src/semubot_eyes/outline.png") # Use an image with a transparent background!
pupil_image = pygame.image.load("src/semubot_eyes/eyebb.png")  # Use an image with a transparent background!
mouth_image = pygame.image.load("src/semubot_eyes/mouth.png") # Use an image with a transparent background!

class RespeakerSubscriber(Node):
    
    def __init__(self):
        super().__init__('subscriber_semubot_eyes')
        
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
        left_eye_x = self.width // 2 - self.eye_distance // 2 - 55 #adjust x eye positions here, default -> 750
        right_eye_x = self.width // 2 + self.eye_distance // 2 + 145 #default -> 2650

        lid_image_scaled = pygame.transform.scale(lid_image, (self.lid_width, self.lid_height))

        outline_image_scaled = pygame.transform.scale(outline_image, (self.lid_width, self.lid_height))

        mouth_image_scaled = pygame.transform.scale(mouth_image, (self.lid_width/3, self.lid_height/6))

        self.screen.blit(lid_image_scaled, (270, 100))
        self.screen.blit(outline_image_scaled, (270, 100))
        self.screen.blit(mouth_image_scaled, (730, 700))
        
        if abs(self.direction) > 120: #lasy control of the borders
            self.direction = self.direction * 0.65

        pupil_x_left = left_eye_x + self.direction 
        pupil_x_right = right_eye_x + self.direction 
        
        #adjust the y-coordinate to center the pupils within the lids
        pupil_y = self.eye_y - self.pupil_radius * 2 - (+65) #adjust y eye position here
        pupil_img = pygame.transform.scale(pupil_image, (self.pupil_radius * 2.95, self.pupil_radius * 2.95))
        self.screen.blit(pupil_img, (pupil_x_left - self.pupil_radius * 2, pupil_y))
        self.screen.blit(pupil_img, (pupil_x_right - self.pupil_radius * 2, pupil_y))

    def doa_callback(self, msg):
        self.get_logger().info('DOA: "%s"' % msg.pose)

    def moving_average(pos, n_values): #can be used for smoothing out the incoming doa data -> slower eye movement
        """
        :param pos: The latest value that is passed to the filter
        :n_values: The integer after which we start popping out the unnecessary values from our array and account only for the newest n_values only
        :return: The filtered value
        """
        global arr, n
        
        if pos:
            arr.append(pos)
            n = n + 1

        if n > n_values:
            arr = arr[-n_values:]
            n = n_values

        if n > 0:
            filtered_val = sum(arr[::-1]) / n
        else:
            filtered_val = 0  # Or any other default value
            
        print("Filter val: ", filtered_val)
            
        return filtered_val

def main(args=None):
    rclpy.init(args=args)
    subscriber = RespeakerSubscriber()

    try:
        while rclpy.ok():
            #process Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
                elif event.type == pygame.VIDEORESIZE:
                    #handle window resize event
                    subscriber.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)

            #process ROS messages
            rclpy.spin_once(subscriber, timeout_sec=0)

            #update display
            subscriber.screen.fill((255, 255, 255))
            subscriber.draw_eyes()
            pygame.display.flip()
            pygame.time.delay(10)

    except KeyboardInterrupt:
        pass

    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
