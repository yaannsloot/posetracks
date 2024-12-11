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

Database functions used to store processed C structures from makesdna headers
"""

import sqlite3

_db = sqlite3.connect('header_data.db')
_cursor = _db.cursor()


def get_tables():
    _cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    return {table[0] for table in _cursor.fetchall()}


def _init_blend_versions():
    tables = get_tables()
    if 'blend_ver' not in tables:
        query = """
                CREATE TABLE blend_ver
                (
                    ver_id integer primary key,
                    major integer,
                    minor integer,
                    patch integer,
                    loaded integer default 0,
                    evaluated integer default 0,
                    unique (major, minor, patch)
                );
                """
        _cursor.execute(query)
        _db.commit()


def _init_structures():
    tables = get_tables()
    if 'structures' not in tables:
        query = """
                CREATE TABLE structures
                (
                    struct_id integer primary key,
                    tag text,
                    unique (tag)
                );
                """
        _cursor.execute(query)
        _db.commit()


def _init_revisions():
    tables = get_tables()
    if 'revisions' not in tables:
        query = """
                CREATE TABLE revisions
                (
                    rev_id integer primary key,
                    struct_id integer,
                    ver_id integer,
                    src text,
                    hash text,
                    available integer,
                    foreign key (struct_id) references structures (struct_id),
                    foreign key (ver_id) references blend_ver (ver_id)
                );
                """
        _cursor.execute(query)
        _db.commit()


def _init_dependencies():
    tables = get_tables()
    if 'dependencies' not in tables:
        query = """
                CREATE TABLE dependencies
                (
                    dep_id integer primary key,
                    rev_id integer,
                    is_ptr integer,
                    foreign key (rev_id) references structures (rev_id)
                );
                """
        _cursor.execute(query)
        _db.commit()


def init_db():
    _init_blend_versions()
    _init_structures()
    _init_revisions()
    _init_dependencies()


def add_version(major, minor, patch):
    query = """
    INSERT OR IGNORE INTO blend_ver (major, minor, patch)
    VALUES (?,?,?);
    """
    _cursor.execute(query, (major, minor, patch))
    _db.commit()


def set_version_loaded(major, minor, patch, flag=False):
    query = """
    UPDATE blend_ver
    SET loaded = ?
    WHERE major = ? AND minor = ? AND patch = ?;
    """
    _cursor.execute(query, (flag, major, minor, patch))
    _db.commit()


def set_version_evaluated(major, minor, patch, flag=False):
    query = """
    UPDATE blend_ver
    SET evaluated = ?
    WHERE major = ? AND minor = ? AND patch = ?;
    """
    _cursor.execute(query, (flag, major, minor, patch))
    _db.commit()


def is_version_loaded(major, minor, patch):
    _cursor.execute("SELECT loaded FROM blend_ver WHERE major = ? AND minor = ? AND patch = ?;",
                    (major, minor, patch))
    ret = _cursor.fetchone()
    if ret is not None:
        ret = bool(ret[0])
    return ret


def get_versions():
    _cursor.execute("SELECT * FROM blend_ver;")
    return {(maj, m, p) for _, maj, m, p, _, _ in _cursor.fetchall()}
