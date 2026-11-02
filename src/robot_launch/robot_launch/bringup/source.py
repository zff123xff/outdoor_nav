"""Lazy in-memory inclusion of a launch component."""
from launch import LaunchDescriptionSource

class ComponentSource(LaunchDescriptionSource):
    def __init__(self, factory):
        self.factory = factory
        super().__init__(location=factory.__module__)

    def _get_launch_description(self, location):
        return self.factory()
