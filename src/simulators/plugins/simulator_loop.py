import pygame
import queue

class SimulatorLoop:
    def __init__(self, runtime):
        self.runtime = runtime
        self.running = True
        self.event_queue = queue.Queue()
        self.clock = pygame.time.Clock()

    def run(self):
        """Run the simulator loop on the main thread"""
        try:
            while self.running:
                if hasattr(self.runtime.config, 'simulators'):
                    for simulator in self.runtime.config.simulators:
                        try:
                            simulator.tick()
                        except Exception as e:
                            print(f"Error in simulator tick: {e}")
                            if "display" in str(e).lower():
                                continue
                            self.running = False
                            break
                
                # Maintain consistent frame rate
                self.clock.tick(60)

        except KeyboardInterrupt:
            self.cleanup()
        except Exception as e:
            print(f"Error in simulator loop: {e}")
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self.runtime.config, 'simulators'):
            for simulator in self.runtime.config.simulators:
                try:
                    simulator.cleanup()
                except Exception as e:
                    print(f"Error cleaning up simulator: {e}") 