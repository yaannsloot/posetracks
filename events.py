'''
Copyright (C) 2024 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

from . import posetracks_core as pt_core


class Event:
    """
    Basic event. Can have an optional response message.
    """

    def __init__(self, msg=''):
        self.msg = msg


class InfoEvent(Event):
    """
    Info event. Can have an additional info message for context specific situations.
    """

    def __init__(self, info_msg='', msg=''):
        super().__init__(msg)
        self.info_msg = info_msg


class ErrorEvent(Event):
    """
    Error event. Same as normal event but can hold an additional error message string.
    """

    def __init__(self, error_msg='', msg=''):
        super().__init__(msg)
        self.error_msg = error_msg


class CancelledEvent(Event):
    def __init__(self, cancel_msg='', msg=''):
        super().__init__(msg)
        self.cancel_msg = cancel_msg


class DetectionFinishedEvent(Event):
    """
    Indicates that a task involving detection operations with the MEPython dnn module have finished.
    """

    def __init__(self, detections: dict[int, dict[int, pt_core.dnn.Detection]], msg=''):
        super().__init__(msg)
        self.detections = detections


class TagFinishedEvent(Event):
    """
    Indicates that a task involving tag operations with the MEPython dnn module have finished.
    """

    def __init__(self, tags: dict[int, dict[int, pt_core.dnn.Tag]], msg=''):
        super().__init__(msg)
        self.tags = tags


class PoseFinishedEvent(Event):
    """
    Indicates that a task involving pose operations with the MEPython dnn module have finished.
    """

    def __init__(self, poses: dict[int, dict[str, pt_core.dnn.Pose]], msg=''):
        super().__init__(msg)
        self.poses = poses


def default_response(event: Event, *args):
    """
    Default event response. Will print the contents of event.msg.
    :param event: Event to respond to
    """
    if event.msg != '':
        print(event.msg)


class EventListener:
    """
    Base event listener. Responds only to basic events.
    """
    expected_type = Event

    def __init__(self, notify_response=default_response):
        self.notify_response = notify_response

    def notify(self, event, *args):
        """
        If the event matches the expected type, will respond by calling self.notify_response.
        :param event: Event to respond to
        :param args: Optional args to pass to self.notify_response
        :return: The return value of self.notify_response, which could be None
        """
        if not type(event) is self.expected_type:
            return
        return self.notify_response(event, *args)


class InfoEventListener(EventListener):
    """
    Info event listener. Responds only to info events.
    """
    expected_type = InfoEvent


class ErrorEventListener(EventListener):
    """
    Error event listener. Responds only to info events.
    """
    expected_type = ErrorEvent


class CancelledEventListener(EventListener):
    expected_type = CancelledEvent


class DetectionFinishedEventListener(EventListener):
    """
    Detection task finished event listener. Responds only to events of the same type.
    """
    expected_type = DetectionFinishedEvent


class TagFinishedEventListener(EventListener):
    """
    Tag task finished event listener. Responds only to events of the same type.
    """
    expected_type = TagFinishedEvent


class PoseFinishedEventListener(EventListener):
    """
    Pose task finished event listener. Responds only to events of the same type.
    """
    expected_type = PoseFinishedEvent


class EventDispatcher:
    """
    Manages event listeners. Useful when using more than one listener for a given task.
    """

    def __init__(self):
        self._listeners = []

    def register_listener(self, listener):
        """
        Registers a new event listener to this event dispatcher.
        If a listener of the same type was already registered,
        it will be replaced.
        :param listener: Listener to register to this dispatcher
        """
        if isinstance(listener, EventListener):
            for i in range(len(self._listeners)):
                if type(listener) is type(self._listeners[i]):
                    self._listeners[i] = listener
                    return
            self._listeners.append(listener)

    def broadcast(self, event: Event, *args):
        """
        Send an event to a matching event listener.
        If a listener that expects the type of event is available,
        this function will notify that listener and return the value of listener.notify()
        :param event: Event to broadcast
        :param args: Optional args to pass to listener's notify()
        :return: The value of the matching listener's notify(), or None
        """
        for listener in self._listeners:
            if isinstance(listener, EventListener) and type(event) is listener.expected_type:
                return listener.notify(event, *args)

