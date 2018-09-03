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


"""Cython support for 3D transformation and command buffer encoding

The matrix transformation of the vertex buffer and encoding into
the command buffer is written in Cython, rather than pure Python,
because we need the speed to hit 60 fps with a sizeable model
when running on a Raspberry Pi.
"""


cimport libc.stdlib


#
#  A homogenous vector
#
cdef struct Vector:

    float x
    float y
    float z
    float w


#
#  A coordinate for an on-screen pixel
#
cdef struct Pixel:
    int x
    int y


#
#  A 4x4 matrix representing a 3D transform
#
cdef struct Matrix:
    float m[16]


#
#  A line on-screen (aka model edge)
#
#  Contains a begin point (bx, by) and end point (ex, ey).
#
cdef struct Edge:
    int bx
    int by
    int ex
    int ey


cdef class VertexBuffer:

    """A buffer containing vertices to be drawn with an index buffer"""

    cdef Vector *verts
    cdef int count

    cdef Pixel *xform  # scratch space for transformed vertices

    def __cinit__(self, pyverts):
        cdef int i

        self.count = len(pyverts)
        self.verts = <Vector *>libc.stdlib.malloc(sizeof(Vector) * self.count)
        self.xform = <Pixel *>libc.stdlib.malloc(sizeof(Pixel) * self.count)

        if not self.verts or not self.xform:
            raise MemoryError()

        for i in range(self.count):
            self.verts[i].x = pyverts[i][0]
            self.verts[i].y = pyverts[i][1]
            self.verts[i].z = pyverts[i][2]
            self.verts[i].w = pyverts[i][3]

    def __dealloc__(self):
        libc.stdlib.free(self.verts)
        libc.stdlib.free(self.xform)
        self.verts = NULL


cdef class IndexBuffer:

    """Indices into a vertex buffer representing a list of model edges"""

    cdef int *indices
    cdef int count
    cdef int min_index, max_index

    def __cinit__(self, pyindices):
        cdef int i

        self.count = 2 * len(pyindices)
        self.indices = <int *>libc.stdlib.malloc(sizeof(int) * self.count)

        if not self.indices:
            raise MemoryError()

        i = 0
        for e in pyindices:
            self.indices[i] = e[0]
            self.indices[i + 1] = e[1]
            i += 2

        self.min_index = 0
        self.max_index = 0
        for i in range(self.count):
            self.min_index = min(self.min_index, self.indices[i])
            self.max_index = max(self.max_index, self.indices[i])

    def __dealloc__(self):
        libc.stdlib.free(self.indices)
        self.indices = NULL


cdef int matrix_to_cmatrix(Matrix *cmatrix, matrix) except 1:

    """Convert a Python matrix tuple of tuples into a C-style matrix"""
    
    cdef int i, j

    for i in range(4):
        for j in range(4):
            cmatrix.m[i * 4 + j] = matrix[i][j]

    return 0


cdef Pixel *transform_vertices(
        Matrix *matrix, VertexBuffer verts) except NULL:

    """Apply a transformation matrix to all vertices in a vertex buffer

    Returns a buffer of transformed screen coordinates.  The returned
    buffer shouldn't be freed -- it comes from the scratch space inside
    the vertex buffer.
    """

    cdef Pixel *xform
    cdef Vector *v
    cdef float *m
    cdef float x, y, z, w
    cdef int i

    xform = verts.xform
    m = matrix.m

    for i in range(verts.count):
        v = &verts.verts[i]

        x = m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3] * v.w
        y = m[4] * v.x + m[5] * v.y + m[6] * v.z + m[7] * v.w
        z = m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11] * v.w
        w = m[12] * v.x + m[13] * v.y + m[14] * v.z + m[15] * v.w

        xform[i].x = <int>(x / w)
        xform[i].y = <int>(y / w)

    return xform


cdef int encode_edges(
        cmd, int cmd_used, Pixel *xform, IndexBuffer indices) except -1:

    """Encode edges from an index buffer into command buffer format"""

    cdef int i
    cdef int a, b
    cdef int bx, by, ex, ey

    i = 0
    while i + 1 < indices.count:
        a = indices.indices[i]
        b = indices.indices[i + 1]
        i += 2

        if xform[b].y < xform[a].y:
            (a, b) = (b, a)

        bx = xform[a].x
        by = xform[a].y
        ex = xform[b].x
        ey = xform[b].y

        cmd[cmd_used] = (bx >> 2) & 0xFF
        cmd[cmd_used + 1] = ((bx << 6) | (by >> 4)) & 0xFF
        cmd[cmd_used + 2] = ((by << 4) | (ex >> 6)) & 0xFF
        cmd[cmd_used + 3] = ((ex << 2) | (ey >> 8)) & 0xFF
        cmd[cmd_used + 4] = ey & 0xFF

        cmd_used += 5

    return cmd_used


def draw_model(cmd, cmd_used, matrix, vertex_buffer, index_buffer):

    """Draw a model

    The vertices from the vertex buffer are transformed, the
    edges are generated from the index buffer, and the results
    are encoded into the command buffer for transmission to
    the GPU."""

    cdef Pixel *xform
    cdef Matrix cmatrix
    cdef int i

    vb = <VertexBuffer?>vertex_buffer
    ib = <IndexBuffer?>index_buffer

    if ib.min_index < 0 or ib.max_index >= vb.count:
        raise ValueError("Index out of Vertex Buffer range")

    matrix_to_cmatrix(&cmatrix, matrix)
    xform = transform_vertices(&cmatrix, vb)
    r = encode_edges(cmd, cmd_used, xform, ib)

    return r
