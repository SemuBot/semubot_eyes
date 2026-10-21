import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import pygame
import os
import random
import time
import math
import re
from semubot_eyes.expressions import ExpressionState
from semubot_eyes.playback import PlaybackTracker


def parse_background_color(value):
    """Accept a #RRGGBB setting and return the RGB tuple used by Pygame."""
    if not re.fullmatch(r"#[0-9A-Fa-f]{6}", value):
        return None
    return tuple(int(value[index:index + 2], 16) for index in (1, 3, 5))

class RespeakerSubscriber(Node):
    def __init__(self):
        super().__init__('eye_controller')
        
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN | pygame.NOFRAME)
        self.width, self.height = 1920, 1440
        pygame.display.set_caption("Semuscreen")
        # Scaling factors
        self.scale_x = self.width / 1920
        self.scale_y = self.height / 1080
        
        # ROS Setup
        eyes_package_name = 'semubot_eyes'
        eyes_package_path = get_package_share_directory(eyes_package_name)
        self.img_dir_path = os.path.join(eyes_package_path, "images")
        self.background_color = (255, 255, 255)
        
        self.subscription_doa_raw = self.create_subscription(Int32, 'doa_raw', self.doa_raw_callback, 10)
        self.subscription_doa = self.create_subscription(PoseStamped, 'doa', self.doa_callback, 10)
        color_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.subscription_background_color = self.create_subscription(
            String, '/face/background_color', self.background_color_callback, color_qos)
        self.activity = ExpressionState()
        self.playback = PlaybackTracker()
        self.expression = 'idle'
        self.expression_since = time.monotonic()
        self.last_update = time.monotonic()
        self.publisher_present = False
        self.next_graph_check = 0.0
        self.subscription_activity = self.create_subscription(
            String, '/semubot/state', self.activity_callback, color_qos)
        self.subscription_log = self.create_subscription(
            String, '/semubot/conversation_log', self.log_callback, color_qos)
        
        #  Load Static Images 
        self.lid_image = self.load_and_scale("both_eyes.png", 1.0, 1.0)
        self.outline_image = self.load_and_scale("outline.png", 1.0, 1.0)
        self.nose_image = self.load_and_scale("nose.png", 1500/1920, 650/1080, custom_dims=True)
        
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
        self.eye_center = (self.width // 2, int((self.height* 0.185) * self.scale_y))
        #self.eye_center = (self.width // 2, self.height // 2 - int(100 * self.scale_y))
        self.pos_lid = (self.eye_center[0] - self.lid_image.get_width() // 2,
                        self.eye_center[1] - self.lid_image.get_height() // 3)
        self.pos_nose = (self.eye_center[0] - self.nose_image.get_width() // 2,
                         int(100 * self.scale_y))
        self.pos_mouth = (int(300 * self.scale_x), int(300 * self.scale_y))
        mouth_bounds = self.mouth_images[0].get_bounding_rect()
        self.mouth_center = (self.pos_mouth[0] + mouth_bounds.centerx,
                             self.pos_mouth[1] + mouth_bounds.centery)
        mouth = self.mouth_images[0].subsurface(mouth_bounds).copy()
        self.closed_mouth = pygame.transform.smoothscale(
            mouth, (mouth.get_width(), max(1, int(mouth.get_height() * 0.16))))
        self.concerned_mouth = pygame.transform.flip(self.closed_mouth, False, True)
        self.closed_mouth = mouth  # smiling default mouth (full-height mouth4.png)
        self.speech_mouths = [image.subsurface(image.get_bounding_rect()).copy()
                             for image in self.mouth_images]
        
        self.current_eye_index = 0
        self.target_eye_index = 0
        self.eye_transition_progress = 1.0
        
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

    def background_color_callback(self, msg):
        color = parse_background_color(msg.data.strip())
        if color is None:
            self.get_logger().warn(f"Ignoring invalid face background color: {msg.data!r}")
            return
        self.background_color = color
        self.get_logger().info(f"Face background color: {msg.data.strip().upper()}")

    def activity_callback(self, msg):
        if not self.activity.receive_state(msg.data):
            self.get_logger().warn('Ignoring invalid robot activity')

    def log_callback(self, msg):
        self.activity.receive_log(msg.data)
    
    def update_logic(self):
        current_time = time.time()
        now = time.monotonic()
        elapsed = max(0.0, min(0.2, now - self.last_update))
        self.last_update = now
        if now >= self.next_graph_check:
            self.publisher_present = self.count_publishers('/semubot/state') > 0
            self.next_graph_check = now + 0.5
        playing = self.playback.update()
        expression = self.activity.expression(self.publisher_present, playing)
        if expression != self.expression:
            self.expression = expression
            self.expression_since = now
            self.current_eye_index = 0
            self.target_eye_index = 0
            self.eye_transition_progress = 1.0
            self.get_logger().info(f'Face expression: {expression}')
        
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
        if self.expression == 'idle' and current_time - self.last_saccade_time > self.next_saccade_delay:
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
            self.eye_transition_progress = min(1.0, self.eye_transition_progress + elapsed / 0.18)
            if self.eye_transition_progress >= 1.0:
                self.current_eye_index = self.target_eye_index

        
        # Audio / Mouth Logic 
        self.current_mouth_index = self.playback.mouth_frame() if playing else 0
    
    def draw_eyes(self):
        if self.is_blinking:
            # Use the closed eye image
            current_eye = self.directional_eyes[self.blink_image_index]
        else:
            # Use the current directional eye
            current_eye = self.directional_eyes[self.current_eye_index]
            
        eye_x = self.eye_center[0] - current_eye.get_width() // 2
        eye_y = self.eye_center[1] - current_eye.get_height() // 3
        if self.expression == 'thinking' and not self.is_blinking:
            phase = time.monotonic() - self.expression_since
            eye_x += round(6 * self.scale_x * math.sin(phase * 1.8))
            eye_y -= round(5 * self.scale_y * (0.5 + 0.5 * math.sin(phase * 1.8)))
        self.screen.blit(current_eye, (eye_x, eye_y))
        if self.expression == 'idle' and not self.is_blinking and self.eye_transition_progress < 1.0:
            target = self.directional_eyes[self.target_eye_index].copy()
            progress = 1 - (1 - self.eye_transition_progress) ** 3
            target.set_alpha(round(progress * 255))
            self.screen.blit(target, (eye_x, eye_y))
        
        # Outline
        self.screen.blit(self.outline_image, self.pos_lid)
        
        # Features
        self.screen.blit(self.nose_image, self.pos_nose)
        self.draw_expression_brows(eye_x, eye_y)
        
        # Mouth
        mouth_img = self.closed_mouth
        if self.expression == 'error':
            mouth_img = self.concerned_mouth
        elif self.expression == 'speaking' and self.current_mouth_index:
            mouth_img = self.speech_mouths[self.current_mouth_index]
        self.screen.blit(mouth_img, mouth_img.get_rect(center=self.mouth_center))
    
    def draw_expression_brows(self, eye_x, eye_y):
        """Draw stable eyebrows above the artwork, independent of image margins."""
        if self.expression not in ('listening', 'thinking', 'error'):
            return
        bounds = self.directional_eyes[0].get_bounding_rect().move(eye_x, eye_y)
        brow_width = bounds.width * 0.24
        base_y = bounds.top - 35 * self.scale_y
        thickness = max(3, round(12 * self.scale_y))
        for side, fraction in enumerate((0.17, 0.83)):
            center_x = bounds.left + bounds.width * fraction
            start_y = end_y = base_y
            lift = 25 * self.scale_y
            if self.expression == 'error':
                if side == 0:
                    end_y -= 35 * self.scale_y
                else:
                    start_y -= 35 * self.scale_y
                lift = 8 * self.scale_y
            elif self.expression == 'thinking' and side == 1:
                start_y -= 14 * self.scale_y
                end_y -= 14 * self.scale_y
            points = []
            for step in range(25):
                t = step / 24
                x = center_x - brow_width / 2 + brow_width * t
                y = start_y * (1 - t) + end_y * t - 4 * lift * t * (1 - t)
                points.append((round(x), round(y)))
            pygame.draw.lines(self.screen, (20, 20, 20), False, points, thickness)
            for point in (points[0], points[-1]):
                pygame.draw.circle(self.screen, (20, 20, 20), point, thickness // 2)


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
            
            eye_controller.screen.fill(eye_controller.background_color)
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
