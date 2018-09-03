#
# Copyright (c) 2018, Matt Kimball
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# The views and conclusions contained in the software and documentation are those
# of the authors and should not be interpreted as representing official policies,
# either expressed or implied, of the ToyGPU project.
#


"""Communication with ToyGPU

We are using both pigpio and RPi.GPIO because we need pigpio's
SPI support for transmissions larger than 4096, and because blocking
on a GPIO falling edge with precise timing is much simpler with
RPi.GPIO than with pigpio.
"""


import pigpio
import RPi.GPIO as GPIO
import toybuffer
import toy3d


VSYNC_PIN = 25
SPI_TRANSFER_SPEED = 4 * 1024 * 1024
COMMAND_BUFFER_SIZE = 8192

_pi = None
_spi = None
_command_buffer = bytearray(COMMAND_BUFFER_SIZE)
_command_buffer_used = 0

_model_matrix = toy3d.matrix_identity()
_view_matrix = toy3d.matrix_identity()
_project_matrix = toy3d.matrix_identity()
_screen_matrix = toy3d.matrix_screen(640, 480)
_mvp_matrix = toy3d.matrix_identity()


def opengpu():
    """Open the connection to the GPU"""

    global _pi
    global _spi

    _pi = pigpio.pi()
    _spi = _pi.spi_open(0, SPI_TRANSFER_SPEED, 0)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(VSYNC_PIN, GPIO.IN)


def closegpu():
    """Close the GPU connection"""

    _pi.spi_close(_spi)
    _pi.stop()


def swap():

    """Send the command buffer to the GPU (over SPI) and wait for VSYNC"""

    global _command_buffer_used

    if not _pi:
        raise IOError("toygpu not open")

    _pi.spi_xfer(_spi, _command_buffer[0:_command_buffer_used])
    _command_buffer_used = 0
    GPIO.wait_for_edge(VSYNC_PIN, GPIO.FALLING)


def _generate_mvp():
    """Generate a combined (model+view+projection) matrix"""

    global _mvp_matrix

    mv = toy3d.matrix_multiply(_view_matrix, _model_matrix)
    mvp = toy3d.matrix_multiply(_project_matrix, mv)
    _mvp_matrix = toy3d.matrix_multiply(_screen_matrix, mvp)

def set_model_matrix(m):
    """Set the model transformation matrix"""

    global _model_matrix

    _model_matrix = m
    _generate_mvp()

def set_view_matrix(m):
    """Set the camera view matrix"""

    global _view_matrix

    _view_matrix = m
    _generate_mvp()

def set_project_matrix(m):
    """Set the projection to screen space matrix"""

    global _project_matrix

    _project_matrix = m
    _generate_mvp()


def draw_model(model):
    """Draw a model with a vertex buffer and index buffer"""

    global _command_buffer_used

    _command_buffer_used = toybuffer.draw_model(
        _command_buffer, _command_buffer_used, _mvp_matrix,
        model.vertex_buffer, model.index_buffer)
