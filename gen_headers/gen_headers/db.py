"""
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

----------------------------------------------------------------------

Database model definitions for storing processed C structures
"""

import zlib
from peewee import *

_db = SqliteDatabase('header_data.db')


def crc32(src):
    return zlib.crc32(src.encode("utf-8"), 0) & 0xffffffff


class BaseModel(Model):
    class Meta:
        database = _db


class BlenderVersion(BaseModel):
    ver_id = IntegerField(primary_key=True)
    major = IntegerField()
    minor = IntegerField()
    patch = IntegerField()
    loaded = BooleanField(default=False)
    evaluated = BooleanField(default=False)

    class Meta:
        indexes = (
            (('major', 'minor', 'patch'), True),
        )


class Struct(BaseModel):
    struct_id = IntegerField(primary_key=True)
    tag = TextField(unique=True)


class Revision(BaseModel):
    rev_id = IntegerField(primary_key=True)
    struct_id = ForeignKeyField(Struct, backref='revisions')
    ver_id = ForeignKeyField(BlenderVersion)
    src = TextField()
    crc32 = IntegerField(unique=True)
    available = BooleanField(default=True)

    class Meta:
        indexes = (
            (('struct_id', 'ver_id'), True),
        )


class Dependency(BaseModel):
    dep_id = IntegerField(primary_key=True)
    rev_id = ForeignKeyField(Revision, backref='dependencies')
    struct_id = ForeignKeyField(Struct)
    is_ptr = BooleanField(default=False)

    class Meta:
        indexes = (
            (('rev_id', 'struct_id'), True),
        )


_db.connect()
_db.create_tables([BlenderVersion, Struct, Revision, Dependency])
