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

_db = SqliteDatabase(None)


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
    src = TextField()
    crc32 = IntegerField(unique=True)


class RevisionVersion(BaseModel):
    ver_id = ForeignKeyField(BlenderVersion)
    rev_id = ForeignKeyField(Revision, backref='versions')
    available = BooleanField(default=True)

    class Meta:
        indexes = (
            (('ver_id', 'rev_id'), True),
        )


class Dependency(BaseModel):
    dep_id = IntegerField(primary_key=True)
    rev_id = ForeignKeyField(Revision, backref='dependencies')
    tag = TextField()
    is_ptr = BooleanField(default=False)

    class Meta:
        indexes = (
            (('rev_id', 'tag', 'is_ptr'), True),
        )


def _get_ranked_rev_cte(ver):
    return (
        Revision.select(
            BlenderVersion.major, BlenderVersion.minor, BlenderVersion.patch,
            Struct.tag,
            RevisionVersion.available, Revision.rev_id,
            fn.ROW_NUMBER().over(
                partition_by=[Struct.tag],
                order_by=[
                    BlenderVersion.major.desc(),
                    BlenderVersion.minor.desc(),
                    BlenderVersion.patch.desc()
                ]
            ).alias('rank')
        ).join(RevisionVersion, on=(Revision.rev_id == RevisionVersion.rev_id))
        .join(BlenderVersion, on=(BlenderVersion.ver_id == RevisionVersion.ver_id))
        .join(Struct, on=(Struct.struct_id == Revision.struct_id))
        .where(
            (BlenderVersion.major < ver[0]) |
            ((BlenderVersion.major == ver[0]) & (BlenderVersion.minor < ver[1])) |
            ((BlenderVersion.major == ver[0]) & (
                BlenderVersion.minor == ver[1]) & (BlenderVersion.patch <= ver[2]))
        )
        .cte('ranked_revisions')
    )


def _get_top_rev_cte(ranked_rev_cte):
    return (
        ranked_rev_cte.select(
            ranked_rev_cte.c.major,
            ranked_rev_cte.c.minor,
            ranked_rev_cte.c.patch,
            ranked_rev_cte.c.tag,
            ranked_rev_cte.c.available,
            ranked_rev_cte.c.rev_id
        ).where(ranked_rev_cte.c.rank == 1)
        .cte('top_revisions')
    )


def _get_target_dep_cte(top_rev_cte):
    other_top_rev = top_rev_cte.alias('tr_b')
    return (
        top_rev_cte.select(
            top_rev_cte.c.major, top_rev_cte.c.minor, top_rev_cte.c.patch,
            top_rev_cte.c.rev_id,
            top_rev_cte.c.tag,
            Revision.src,
            Dependency.tag.alias('dep_tag'),
            other_top_rev.c.rev_id.alias('dep_id'),
            Dependency.is_ptr.alias('dep_is_ptr'),
            other_top_rev.c.major.alias('dep_major'),
            other_top_rev.c.minor.alias('dep_minor'),
            other_top_rev.c.patch.alias('dep_patch'),
            Case(None, (
                (Dependency.tag.is_null(), None),
                (other_top_rev.c.available == 1, 1)
            ), 0).alias('dep_available')
        )
        .join(Revision, on=(Revision.rev_id == top_rev_cte.c.rev_id))
        .join(Dependency, JOIN.LEFT_OUTER, on=(Dependency.rev_id == Revision.rev_id))
        .join(SQL(f'"{top_rev_cte._alias}" AS "tr_b"'), JOIN.LEFT_OUTER, on=(other_top_rev.c.tag == Dependency.tag))
        .cte('target_dependency_table')
    )


def _get_target_rev_available_cte(target_dep_cte):
    return (
        target_dep_cte.select(
            target_dep_cte.c.tag,
            (fn.SUM((target_dep_cte.c.dep_available.is_null()) | (target_dep_cte.c.dep_available == 1) | (
                target_dep_cte.c.dep_is_ptr == 1)) == fn.COUNT(1)).alias('rev_available')
        ).group_by(target_dep_cte.c.tag)
        .cte('target_rev_available')
    )


def debug_cte(primary_cte, dependencies):
    return (
        primary_cte.select()
        .with_cte(*dependencies)
    )


def resolve_dependencies(ver):
    ranked_revisions = _get_ranked_rev_cte(ver)
    top_revisions = _get_top_rev_cte(ranked_revisions)
    target_dep_table = _get_target_dep_cte(top_revisions)
    available_revs = _get_target_rev_available_cte(target_dep_table)
    query = (
        target_dep_table.select(
            target_dep_table.c.major,
            target_dep_table.c.minor,
            target_dep_table.c.patch,
            target_dep_table.c.rev_id,
            target_dep_table.c.tag,
            target_dep_table.c.src,
            target_dep_table.c.dep_tag,
            target_dep_table.c.dep_id,
            target_dep_table.c.dep_available,
            target_dep_table.c.dep_is_ptr,
            target_dep_table.c.dep_major,
            target_dep_table.c.dep_minor,
            target_dep_table.c.dep_patch,
            available_revs.c.rev_available
        )
        .join(available_revs, on=(target_dep_table.c.tag == available_revs.c.tag))
        .with_cte(ranked_revisions, top_revisions, target_dep_table, available_revs)
    )
    return list(query.bind(_db).dicts())


def get_ver_ref(ver: tuple[int, int, int]):
    db_ver, _ = BlenderVersion.get_or_create(
        major=ver[0], minor=ver[1], patch=ver[2])
    return db_ver


def add_dependency(revision, tag, is_ptr):
    db_dep, _ = Dependency.get_or_create(
        rev_id=revision, tag=tag, is_ptr=is_ptr)
    return db_dep


def add_revision(ver, s_def):
    db_ver = get_ver_ref(ver)
    db_struct, _ = Struct.get_or_create(tag=s_def.name)
    src_str = str(s_def)
    try:
        db_rev = Revision.create(struct_id=db_struct, src=src_str,
                                 crc32=crc32(src_str))
        RevisionVersion.get_or_create(ver_id=db_ver, rev_id=db_rev)
        return db_rev
    except IntegrityError:
        return


def single_val_update_atomic(db_obj, sel_attr_dict, attr_val_dict):
    with _db.atomic():
        condition = None
        for k, v in sel_attr_dict.items():
            sub_condition = getattr(db_obj, k).in_(v)
            condition = sub_condition if condition is None else condition & sub_condition
        objs = db_obj.select()
        if condition is not None:
            objs = objs.where(condition)
        for obj in objs:
            for k, v in attr_val_dict.items():
                setattr(obj, k, v)
        db_obj.bulk_update(objs, fields=list(
            attr_val_dict.keys()), batch_size=50)


def tag_revisions_with_ver_atomic(revisions, ver):
    with _db.atomic():
        db_ver = get_ver_ref(ver)
        values = [{"ver_id": db_ver.ver_id, "rev_id": rev.rev_id}
                  for rev in revisions]
        RevisionVersion.insert_many(values).on_conflict_ignore().execute()


def load_revisions_atomic(ver, s_defs):
    with _db.atomic():
        for tag, s_def in s_defs.items():
            rev = add_revision(ver, s_def)
            if rev is None:
                continue
            for line in s_def.body:
                if not line.is_struct:
                    continue
                add_dependency(rev, line.struct_name, line.ptr_level > 0)


def get_version_mappings():
    revs = (Revision.select(
        BlenderVersion.major,
        BlenderVersion.minor,
        BlenderVersion.patch,
        Struct.tag
    ).join(Struct, on=(Revision.struct_id == Struct.struct_id))
        .join(RevisionVersion, on=(Revision.rev_id == RevisionVersion.rev_id))
        .join(BlenderVersion, on=(RevisionVersion.ver_id == BlenderVersion.ver_id))
        .where(RevisionVersion.available == 1).dicts())
    result = {}
    for rev in revs:
        vers = result.setdefault(rev['tag'], set())
        vers.add((rev['major'], rev['minor'], rev['patch']))
    return result


def get_active_versions():
    vers = (BlenderVersion.select(
            BlenderVersion.major,
            BlenderVersion.minor,
            BlenderVersion.patch
            ).distinct()
            .join(RevisionVersion, on=(RevisionVersion.ver_id == BlenderVersion.ver_id))
            .where(RevisionVersion.available == 1).tuples()
            )
    return list(vers)


def _close_db():
    if not _db.is_closed():
        _db.close()


def _init_db(db_file):
    _close_db()
    _db.init(db_file)
    _db.connect()
    _db.create_tables([BlenderVersion, Struct, Revision,
                      RevisionVersion, Dependency])


_init_db('header_data.db')
