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


"""3D math functions for ToyGPU data"""


import math
import toybuffer


def vector_add(a, b):
    """Vector addtion"""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3])

def vector_sub(a, b):
    """Vector subtraction"""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3])

def vector_scale(f, v):
    """Scale a vector by a constant"""
    return (f * v[0], f * v[1], f * v[2], v[3])

def vector_length2(v):
    """Returns the length of a vector squared"""
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]

def vector_length(v):
    """Returns the length of a vector"""
    return math.sqrt(vector_length2(v))

def vector_normalize(v):
    """Normalize a vector (return length one vector in the same direction)"""

    rlen = 1.0 / vector_length(v)
    return (rlen * v[0], rlen * v[1], rlen * v[2], v[3])

def vector_cross(a, b):
    """Cross product of vectors"""

    x = a[1] * b[2] - a[2] * b[1]
    y = a[2] * b[0] - a[0] * b[2]
    z = a[0] * b[1] - a[1] * b[0]

    return (x, y, z, a[3] * b[3])

def vector_dot(a, b):
    """Dot product of vectors"""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]

def vector_homogenize(v):
    """Project a homogenous coordinate (divide x,y,z by w)"""

    rw = 1.0 / v[3]

    return (rw * v[0], rw * v[1], rw * v[2], 1.0)


def vector_transform(m, v):
    """Transform a vector by a 4x4 matrix"""

    x = m[0][0] * v[0] + m[0][1] * v[1] + \
        m[0][2] * v[2] + m[0][3] * v[3]
    y = m[1][0] * v[0] + m[1][1] * v[1] + \
        m[1][2] * v[2] + m[1][3] * v[3]
    z = m[2][0] * v[0] + m[2][1] * v[1] + \
        m[2][2] * v[2] + m[2][3] * v[3]
    w = m[3][0] * v[0] + m[3][1] * v[1] + \
        m[3][2] * v[2] + m[3][3] * v[3]

    return (x, y, z, w)

def matrix_multiply(a, b):
    """Multiply matrices"""

    m = ([0.0, 0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0, 0.0])

    for i in range(4):
        for j in range(4):
            for k in range(4):
                m[i][j] += a[i][k] * b[k][j]

    return m

def matrix_identity():
    """Construct an identity matrix (no transform)"""

    return ((1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))

def matrix_translate(v):
    """Move as directed by a vector"""

    return ((1.0, 0.0, 0.0, v[0]),
            (0.0, 1.0, 0.0, v[1]),
            (0.0, 0.0, 1.0, v[2]),
            (0.0, 0.0, 0.0, 1.0))

def matrix_basis(x, y, z):
    """Rotate to a new set of basis vectors"""

    return ((x[0], x[1], x[2], x[3]),
            (y[0], y[1], y[2], y[3]),
            (z[0], z[1], z[2], z[3]),
            (0.0, 0.0, 0.0, 1.0))

def matrix_rotate_x(angle):
    """Rotate around the x axis"""

    cos = math.cos(angle)
    sin = math.sin(angle)

    return ((1.0, 0.0, 0.0, 0.0),
            (0.0, cos, -sin, 0.0),
            (0.0, sin, cos, 0.0),
            (0.0, 0.0, 0.0, 1.0))

def matrix_rotate_y(angle):
    """Rotate around the y axis"""

    cos = math.cos(angle)
    sin = math.sin(angle)

    return ((cos, 0.0, -sin, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (sin, 0.0, cos, 0.0),
            (0.0, 0.0, 0.0, 1.0))

def matrix_rotate_z(angle):
    """Rotate around the z axis"""

    cos = math.cos(angle)
    sin = math.sin(angle)

    return ((cos, -sin, 0.0, 0.0),
            (sin, cos, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))

def matrix_scale(v):
    """Scale by the components of a vector"""

    return ((v[0], 0.0, 0.0, 0.0),
            (0.0, v[1], 0.0, 0.0),
            (0.0, 0.0, v[2], 0.0),
            (0.0, 0.0, 0.0, v[3]))

def matrix_project(near, far, fov, aspect_ratio):
    """Project from camera space to screen space"""

    s = math.tan(0.5 * fov)

    return ((s, 0.0, 0.0, 0.0),
            (0.0, s * aspect_ratio, 0.0, 0.0),
            (0.0, 0.0, -far / (far - near), -far * near / (far - near)),
            (0.0, 0.0, -1.0, 0.0))

def matrix_view(camera, look_at, up):
    """Transfrom from world space to camera space"""

    z = vector_normalize(vector_sub(camera, look_at))
    x = vector_cross(vector_normalize(up), z)
    y = vector_cross(z, x)

    t = matrix_translate(vector_scale(-1.0, camera))
    r = matrix_basis(x, y, z)

    return matrix_multiply(r, t)

def matrix_screen(x, y):
    """Transform from screen space (-1 to 1) to pixel coordinates"""

    return ((x / 2.0, 0.0, 0.0, x / 2.0 - 0.5),
            (0.0, -y / 2.0, 0.0, y / 2.0 - 0.5),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))


class Model:
    """A 3D model with a vertex buffer and index buffer pair"""

    def __init__(self, verts, edges):
        self.vertex_buffer = toybuffer.VertexBuffer(verts)
        self.index_buffer = toybuffer.IndexBuffer(edges)


def import_obj(filename):

    """Import a model from a Wavefront (.obj) format file"""

    verts = []
    edges = set()

    f = open(filename, 'r')

    for line in f.readlines():
        tokens = line.split(' ')

        if tokens[0] == 'v':
            v = (float(tokens[1]), float(tokens[2]), float(tokens[3]), 1.0)
            verts.append(v)

        if tokens[0] == 'f':
            indices = []
            for v in tokens[1:]:
                indices.append(int(v.split('/')[0]) - 1)

            for i in range(len(indices)):
                a = indices[i]
                b = indices[(i + 1) % len(indices)]

                if a <= b:
                    edge = (a, b)
                else:
                    edge = (b, a)

                edges.add(edge)

    f.close()

    return Model(verts, list(edges))
