"""Exercise the actual ROS subscriptions and renderer on a dummy SDL display."""

import json
import hashlib
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import pygame
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from semubot_eyes.eye_controller import RespeakerSubscriber


class RenderIntegrationTests(unittest.TestCase):
    def test_ros_activity_color_and_every_expression(self):
        rclpy.init()
        pygame.init()
        pygame.display.set_mode((1920, 1440))
        source = rclpy.create_node('face_expression_integration_test')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        activity = source.create_publisher(String, '/semubot/state', qos)
        color = source.create_publisher(String, '/face/background_color', qos)
        activity.publish(String(data='{"stage":"listening"}'))
        color.publish(String(data='#DDF4FF'))
        real_set_mode = pygame.display.set_mode
        # SDL's dummy fullscreen mode is fixed at 1024x768; emulate the actual face screen.
        with patch('pygame.display.set_mode', side_effect=lambda *_args: real_set_mode((1920, 1440))):
            eyes = RespeakerSubscriber()
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(eyes, timeout_sec=0.05)
                eyes.update_logic()
                if eyes.expression == 'listening' and eyes.background_color == (221, 244, 255):
                    break
            self.assertEqual(eyes.expression, 'listening')
            self.assertEqual(eyes.background_color, (221, 244, 255))
            states = [('manual', 'idle'), ('listening', 'listening'),
                      ('thinking', 'thinking'), ('speaking', 'speaking'), ('error', 'error')]
            images = {}
            for stage, expected in states:
                activity.publish(String(data=json.dumps({'stage': stage})))
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    rclpy.spin_once(eyes, timeout_sec=0.05)
                    eyes.update_logic()
                    if eyes.expression == expected:
                        break
                self.assertEqual(eyes.expression, expected)
                eyes.is_blinking = False
                eyes.current_mouth_index = 3 if expected == 'speaking' else 0
                eyes.screen.fill(eyes.background_color)
                eyes.draw_eyes()
                self.assertEqual(tuple(eyes.screen.get_at((0, 0)))[:3], eyes.background_color)
                images[expected] = eyes.screen.copy()
                pygame.image.save(eyes.screen, '/tmp/face-' + expected + '-implemented.png')
            def digest(image):
                return hashlib.sha256(pygame.image.tostring(image, 'RGB')).hexdigest()
            self.assertNotEqual(digest(images['listening']), digest(images['error']))
            self.assertNotEqual(digest(images['idle']), digest(images['speaking']))
            sheet = pygame.Surface((1600, 270))
            sheet.fill((221, 244, 255))
            font = pygame.font.Font(None, 26)
            for i, name in enumerate(('idle', 'listening', 'thinking', 'speaking', 'error')):
                sheet.blit(pygame.transform.smoothscale(images[name], (320, 240)), (i * 320, 25))
                sheet.blit(font.render(name, True, (20, 20, 20)), (i * 320 + 15, 3))
            pygame.image.save(sheet, '/tmp/face-expressions-implemented.png')
            # A historical error snapshot must not turn a healthy face into an error.
            eyes.log_callback(SimpleNamespace(data=json.dumps([
                {'kind': 'error', 'time': time.time() - 60, 'text': 'historical'}])))
            eyes.expression = 'thinking'
            eyes.expression_since = 0
            thinking_frames = []
            for now in (0.0, 0.9):
                with patch('time.monotonic', return_value=now):
                    eyes.screen.fill(eyes.background_color)
                    eyes.draw_eyes()
                    thinking_frames.append(digest(eyes.screen))
            self.assertNotEqual(*thinking_frames)
            eyes.expression = 'idle'
            eyes.is_blinking = True
            eyes.screen.fill(eyes.background_color)
            eyes.draw_eyes()
            self.assertNotEqual(digest(images['idle']), digest(eyes.screen))
            eyes.is_blinking = False
            activity.publish(String(data='{"stage":"manual"}'))
            deadline = time.monotonic() + 3
            while eyes.activity.stage != 'manual' and time.monotonic() < deadline:
                rclpy.spin_once(eyes, timeout_sec=0.05)
                eyes.update_logic()
            self.assertEqual(eyes.activity.stage, 'manual')
            source.destroy_publisher(activity)
            deadline = time.monotonic() + 6
            while eyes.expression != 'error' and time.monotonic() < deadline:
                rclpy.spin_once(eyes, timeout_sec=0.05)
                eyes.update_logic()
            self.assertEqual(eyes.expression, 'error')
            activity = source.create_publisher(String, '/semubot/state', qos)
            activity.publish(String(data='{"stage":"listening"}'))
            deadline = time.monotonic() + 3
            while eyes.expression != 'listening' and time.monotonic() < deadline:
                rclpy.spin_once(eyes, timeout_sec=0.05)
                eyes.update_logic()
            self.assertEqual(eyes.expression, 'listening')
        finally:
            eyes.destroy_node()
            source.destroy_node()
            pygame.quit()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
