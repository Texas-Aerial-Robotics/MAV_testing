import cv2
import numpy as np
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video:
    def __init__(self, port=5600):
        Gst.init(None)
        self.port = port
        self._frame = None

        # Simpler pipeline configuration
        self.video_source = f'udpsrc port={self.port}'
        self.video_codec = '! application/x-rtp ! rtph264depay ! avdec_h264'
        self.video_decode = '! videoconvert ! video/x-raw,format=BGR'
        self.video_sink_conf = '! appsink name=appsink0 emit-signals=true sync=false'

        self.video_pipe = None
        self.video_sink = None
        self.run()

    def start_gst(self, config=None):
        if not config:
            config = [
                'videotestsrc ! decodebin',
                '! videoconvert ! video/x-raw,format=BGR',
                '! appsink'
            ]
        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8
        )
        return array

    def frame(self):
        return self._frame

    def frame_available(self):
        return self._frame is not None

    def run(self):
        self.start_gst([
            self.video_source,
            self.video_codec,
            self.video_decode,
            self.video_sink_conf
        ])
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK