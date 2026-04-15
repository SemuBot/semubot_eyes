import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import pygame
import numpy as np
import scipy.io.wavfile as wav
import subprocess
import os
import random
import time
import math

class RespeakerSubscriber(Node):
    def __init__(self):
        super().__init__('eye_controller')
        
        self.width, self.height = 1024, 600
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN | pygame.NOFRAME)
        pygame.display.set_caption("Semuscreen")
        # Scaling factors
        self.scale_x = self.width / 1920
        self.scale_y = self.height / 1080
        
        # ROS Setup
        eyes_package_name = 'semubot_eyes'
        eyes_package_path = get_package_share_directory(eyes_package_name)
        self.img_dir_path = os.path.join(eyes_package_path, "images")
        
        self.subscription_doa_raw = self.create_subscription(Int32, 'doa_raw', self.doa_raw_callback, 10)
        self.subscription_doa = self.create_subscription(PoseStamped, 'doa', self.doa_callback, 10)
        
        #  Load Static Images 
        self.lid_image = self.load_and_scale("both_eyes.png", 1.0, 1.0)
        self.outline_image = self.load_and_scale("outline.png", 1.0, 1.0)
        self.nose_image = self.load_and_scale("nose.png", 1500/1920, 650/1080, custom_dims=True)
        self.eyebrow_image = self.load_and_scale("eyebrows.png", 2100/1920, 1400/1080, custom_dims=True)
        
        #  Load Mouth Images
        mouth_w_ratio = (2388 * (1 * 320/2388) * 2.135 * 2) / 1920
        mouth_h_ratio = (1668 * (1 * 320/2388) * 2 * 2) / 1080
        self.mouth_images = [
            self.load_and_scale("mouth4.png", mouth_w_ratio, mouth_h_ratio, custom_dims=True),
            #self.load_and_scale("mouth.png", mouth_w_ratio, mouth_h_ratio, custom_dims=True),
            self.load_and_scale("mouth2.png", mouth_w_ratio, mouth_h_ratio, custom_dims=True),
            self.load_and_scale("mouth4.png", mouth_w_ratio, mouth_h_ratio, custom_dims=True),
            self.load_and_scale("mouth5.png", mouth_w_ratio, mouth_h_ratio, custom_dims=True)
        ]
        
        # --- Load Directional Eye Images ---
        self.eye_filenames = [
            "IMG_6479.PNG",  # CENTER
            "IMG_6480.PNG",
            "IMG_6481.PNG",
            "IMG_6482.PNG",
            "IMG_6483.PNG",
            "IMG_6484.PNG",
            "IMG_6485.PNG",
            "IMG_6486.PNG",
            "IMG_7895.PNG"   # CLOSED EYES 
        ]
        
        self.directional_eyes = []
        for fname in self.eye_filenames:
            try:
                img = self.load_and_scale(fname, 1.0, 1.0)
            except:
                self.get_logger().warn(f"Could not load {fname}, using eyebb.png")
                img = self.load_and_scale("eyebb.png", 1.0, 1.0)
            self.directional_eyes.append(img)
            
        self.blink_image_index = len(self.directional_eyes) - 1
        
        self.eye_center = (self.width // 2, int((self.height // 2 - 100) * self.scale_y))
        self.pos_lid = (self.eye_center[0] - self.lid_image.get_width() // 2,
                        self.eye_center[1] - self.lid_image.get_height() // 3)
        self.pos_nose = (self.eye_center[0] - self.nose_image.get_width() // 2,
                         int(100 * self.scale_y))
        self.pos_brows = (self.eye_center[0] - self.eyebrow_image.get_width() // 2,
                          self.eye_center[1] - int(450 * self.scale_y))
        self.pos_mouth = (int(300 * self.scale_x), int(300 * self.scale_y))
        
        self.current_eye_index = 0
        self.target_eye_index = 0
        self.eye_transition_progress = 1.0
        self.eye_transition_speed = 0.15
        
        self.last_saccade_time = time.time()
        self.next_saccade_delay = self.get_natural_saccade_delay()
        self.last_target = 0
        self.movement_history = [0]
        
        self.is_blinking = False
        self.blink_duration = 0.15  # How long eyes stay closed (seconds)
        self.blink_end_time = 0.0
        # Start the first blink countdown
        self.next_blink_time = time.time() + random.uniform(2.0, 5.0)

        # Mouth Logic
        self.current_mouth_index = 0
        self.frame_count = 0
        self.current_wav_file = None
    
    def get_natural_saccade_delay(self):
        if random.random() < 0.5:
            return random.uniform(5.0, 8.0)
        else:
            return random.uniform(3.0, 5.0)
    
    def choose_next_eye_position(self):
        num_eyes = len(self.eye_filenames) - 1 
        
        if random.random() < 0.6:
            return 0
        
        available_positions = list(range(1, num_eyes))
        if self.last_target != 0 and self.last_target in available_positions:
            available_positions.remove(self.last_target)
        
        if len(self.movement_history) > 1:
            recent_positions = self.movement_history[-3:]
            weights = []
            for pos in available_positions:
                if pos in recent_positions:
                    weights.append(0.3)
                else:
                    weights.append(1.0)
            
            if sum(weights) > 0:
                return random.choices(available_positions, weights=weights)[0]
        
        return random.choice(available_positions)
    
    def load_and_scale(self, filename, scale_w, scale_h, custom_dims=False):
        path = os.path.join(self.img_dir_path, filename)
        try:
            img = pygame.image.load(path)
            if custom_dims:
                new_w = int(img.get_width() * self.scale_x)
                new_h = int(img.get_height() * self.scale_y)
                new_w = int(self.width * scale_w) if scale_w <= 1.0 else int(scale_w)
                new_h = int(self.height * scale_h) if scale_h <= 1.0 else int(scale_h)
            else:
                new_w = int(self.width * scale_w)
                new_h = int(self.height * scale_h)
            return pygame.transform.scale(img, (new_w, new_h))
        except Exception as e:
            self.get_logger().error(f"Failed to load image {filename}: {e}")
            return pygame.Surface((100, 100))
    
    def doa_raw_callback(self, msg):
        pass
    
    def doa_callback(self, msg):
        pass
    
    def update_logic(self):
        current_time = time.time()
        
        # Blink 
        # Check if it's time to start a blink
        if not self.is_blinking and current_time > self.next_blink_time:
            self.is_blinking = True
            self.blink_end_time = current_time + self.blink_duration
            #Next blinK: (random interval between 3 to 6 seconds)
            self.next_blink_time = current_time + random.uniform(3.0, 6.0)
            
        # Check if it's time to stop blinking
        if self.is_blinking and current_time > self.blink_end_time:
            self.is_blinking = False

        #  Eye Movement 
        if current_time - self.last_saccade_time > self.next_saccade_delay:
            if self.eye_transition_progress >= 1.0:
                self.target_eye_index = self.choose_next_eye_position()
                self.last_target = self.target_eye_index
                self.movement_history.append(self.target_eye_index)
                if len(self.movement_history) > 5:
                    self.movement_history.pop(0)
                
                self.eye_transition_progress = 0.0
                self.last_saccade_time = current_time
                self.next_saccade_delay = self.get_natural_saccade_delay()
        
        if self.eye_transition_progress < 1.0:
            self.eye_transition_progress = min(1.0, self.eye_transition_progress + self.eye_transition_speed)
            t = self.eye_transition_progress
            eased_progress = 1 - pow(1 - t, 3)
            
            if eased_progress > 0.5:
                self.current_eye_index = self.target_eye_index
        
        # Audio / Mouth Logic 
        wav_file = self.is_wav_playing()
        if wav_file:
            if self.current_wav_file != wav_file:
                self.current_wav_file = wav_file
            if self.frame_count % 5 == 0:
                self.current_mouth_index = (self.current_mouth_index + 1) % len(self.mouth_images)
            self.frame_count += 1
        else:
            self.current_mouth_index = 0
            self.frame_count = 0
    
    def draw_eyes(self):
        if self.is_blinking:
            # Use the closed eye image
            current_eye = self.directional_eyes[self.blink_image_index]
        else:
            # Use the current directional eye
            current_eye = self.directional_eyes[self.current_eye_index]
            
        eye_x = self.eye_center[0] - current_eye.get_width() // 2
        eye_y = self.eye_center[1] - current_eye.get_height() // 3
        self.screen.blit(current_eye, (eye_x, eye_y))
        
        # Outline
        self.screen.blit(self.outline_image, self.pos_lid)
        
        # Features
        self.screen.blit(self.nose_image, self.pos_nose)
        self.screen.blit(self.eyebrow_image, self.pos_brows)
        
        # Mouth
        mouth_img = self.mouth_images[self.current_mouth_index]
        self.screen.blit(mouth_img, self.pos_mouth)
    
    def is_wav_playing(self):
        try:
            result = subprocess.run(['pgrep', '-af', 'play.*\\.wav'], stdout=subprocess.PIPE)
            output = result.stdout.decode()
            if len(output) > 0:
                for line in output.splitlines():
                    if '.wav' in line:
                        return line.split()[-1]
        except Exception:
            pass
        return None


def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    
    eye_controller = RespeakerSubscriber()
    
    try:
        clock = pygame.time.Clock()
        while rclpy.ok():
            for event in pygame.event.get(): 
                if event.type == pygame.QUIT: 
                    pygame.quit() 
                    exit() 
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: 
                    pygame.quit() 
                    exit()
            
            rclpy.spin_once(eye_controller, timeout_sec=0)
            eye_controller.update_logic()
            
            eye_controller.screen.fill((255, 255, 255))
            eye_controller.draw_eyes()
            
            pygame.display.flip()
            clock.tick(30)
    
    except KeyboardInterrupt:
        pass
    finally:
        eye_controller.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()