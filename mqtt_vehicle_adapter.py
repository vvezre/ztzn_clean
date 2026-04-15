# coding=utf-8
"""
?????????? MQTT ??????? Flask HTTP ???
????? requests??????? Python 2 ???????????
"""
import os

try:
    from urllib import urlencode
    from urllib2 import URLError, urlopen
except ImportError:
    from urllib.error import URLError
    from urllib.parse import urlencode
    from urllib.request import urlopen

from AppLogger import logger


class VehicleControllerAdapter(object):
    def __init__(self, base_url=None, timeout=10):
        self.base_url = (base_url or os.environ.get('CLEANER_HTTP_BASE_URL') or 'http://127.0.0.1:7899').rstrip('/')
        self.timeout = timeout
        logger.info('MQTT????????????base_url={}'.format(self.base_url))

    def _call(self, path, params=None):
        url = self.base_url + path
        if params:
            url = url + '?' + urlencode(params)
        logger.warning('MQTT?????????: {} params={}'.format(url, params))
        try:
            response = urlopen(url, timeout=self.timeout)
            return response.read()
        except URLError as exc:
            raise RuntimeError('MQTT adapter request failed: {}'.format(exc))

    def drive(self, distance=0, speed=None):
        return self._call('/vehicle/drive')

    def back(self, distance=0, speed=None):
        return self._call('/vehicle/back')

    def turn_left(self, angle=90):
        if angle == 90:
            return self._call('/vehicle/turnLeft90')
        return self._call('/vehicle/turnLeft')

    def turn_right(self, angle=90):
        if angle == 90:
            return self._call('/vehicle/turnRight90')
        if angle == 180:
            return self._call('/vehicle/turnRight180')
        return self._call('/vehicle/turnRight')

    def stop(self):
        return self._call('/vehicle/parking')

    def parking(self):
        return self._call('/vehicle/parking')

    def joystick_move(self, distance, dir_x, dir_y):
        return self._call('/vehicle/joystickMove/{}/{}/{}'.format(distance, dir_x, dir_y))

    def auto_drive(self):
        return self._call('/vehicle/autoDrive')

    def go_on(self):
        return self._call('/vehicle/goOn')

    def return_to_point(self):
        return self._call('/vehicle/returnToPoint')

    def enter_garage(self):
        return self._call('/vehicle/enterGarage')

    def exit_garage(self):
        return self._call('/vehicle/exitGarage')

    def adjust_speed(self, speed):
        return self._call('/vehicle/adjustSpeed/{}'.format(int(speed)))

    def adjust_brush_speed(self, speed):
        return self._call('/vehicle/adjustBrushSpeed/{}'.format(int(speed)))

    def toggle_tracking(self, enabled):
        tracking = '0' if enabled else '1'
        return self._call('/vehicle/toggleTracking/{}'.format(tracking))

    def toggle_path_planning(self, path):
        return self._call('/vehicle/togglePathPlanning/{}'.format(path))

    def create_task(self, *args, **kwargs):
        raise NotImplementedError('???????? MQTT ??????')

    def select_task(self, *args, **kwargs):
        raise NotImplementedError('???????? MQTT ??????')

    def save_task(self, *args, **kwargs):
        raise NotImplementedError('???????? MQTT ??????')

    def save_params(self, *args, **kwargs):
        raise NotImplementedError('???????? MQTT ??????')

    def get_status(self):
        return self._call('/vehicle/getInfo')
