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

-----------------------------------------------------------------------

Armature mappings for compatible pose triangulation results

FORMAT:
At each level you MUST define the following key value pair

name: str

This is the name of the bone to be generated

tail: int or tuple of joint ids

This represents the target the bone should point to.
The actual length of the bone will be determined by averaging length
on symmetrical targets (children with a . in the name and a l or r suffix)
or by the length of non-symmetrical targets. Note that the bone will not
actually be constrained to point to this target.

tail_direction: enum string in {'X', 'Y', 'Z', '-X', '-Y', '-Z'}

This is the direction the tail of the bone will point on initial armature creation
!! AXIS IN THIS CONTEXT IS POSE SPACE AXIS !!

Optionally, you can also define the following key-value pairs

head: int or tuple of joint ids

If the bone should not be connected to the parent, you would define
the starting location here. If the bone is symmetrical, an average
of the two locations will be determined relative to the parent.
!! THIS IS REQUIRED ON THE ROOT BONE !!

head_direction: enum string in {'X', 'Y', 'Z', '-X', '-Y', '-Z'}
!! AXIS IN THIS CONTEXT IS POSE SPACE AXIS !!

If head is defined and bone is a child, this must also be defined.
Represents direction of separation from parent on initial armature creation

constraints: array containing the following items
int = Copy location target
tuple(int, str) = damped track to target with axis enum {'X', 'Y', 'Z', '-X', '-Y', '-Z'}
tuple(int, str, str) = locked track to target with lock axis {'X', 'Y', 'Z'}
and track axis {'X', 'Y', 'Z', '-X', '-Y', '-Z'}
!! AXIS IN THIS CONTEXT IS BONE AXIS !!

These will be applied in order and with influence set similar to the behavior of the
constraint tools panel. This means that every constraint of the same type will have a
diminishing influence to average their results.

rotation: same as constraints
HOWEVER constraints will instead be applied to an axis empty and the resulting rotations
will be copied via a copy rotation constraint. You can still define regular constraints
but this will be applied first.

'''

# Axis definitions
X = 'X'
Y = 'Y'
Z = 'Z'
N_X = '-X'
N_Y = '-Y'
N_Z = '-Z'

AXIS_DIRECTIONS = [X, Y, Z, N_X, N_Y, N_Z]
AXIS_LIST = [X, Y, Z]

# Mappings
person17 = {
    'name': 'Hips',
    'head': (11, 12),
    'tail': (5, 6, 11, 12),
    'tail_direction': Z,
    'constraints': [
        11, 12,
        (11, X), (12, N_X),
        (5, X, Y), (6, X, Y)
    ],
    'children': [
        {
            'name': 'Chest',
            'tail': (5, 6),
            'tail_direction': Z,
            'constraints': [
                (5, Y), (6, Y),
                (5, Y, X), (6, Y, N_X)
            ],
            'children': [
                {
                    'name': 'Head',
                    'tail': (3, 4),
                    'tail_direction': Z,
                    'rotation': [
                        3, 4,
                        (5, N_Y), (6, N_Y),
                        (0, Y, Z), (1, Y, Z), (2, Y, Z)
                    ]
                },
                {
                    'name': 'Shoulder.L',
                    'tail': 5,
                    'tail_direction': X,
                    'constraints': [
                        (5, Y)
                    ],
                    'children': [
                        {
                            'name': 'Arm1.L',
                            'tail': 7,
                            'tail_direction': X,
                            'constraints': [
                                (7, Y)
                            ],
                            'children': [
                                {
                                    'name': 'Arm2.L',
                                    'tail': 9,
                                    'tail_direction': X,
                                    'constraints': [
                                        (9, Y)
                                    ]
                                }
                            ]
                        }
                    ]
                },
                {
                    'name': 'Shoulder.R',
                    'tail': 6,
                    'tail_direction': N_X,
                    'constraints': [
                        (6, Y)
                    ],
                    'children': [
                        {
                            'name': 'Arm1.R',
                            'tail': 8,
                            'tail_direction': N_X,
                            'constraints': [
                                (8, Y)
                            ],
                            'children': [
                                {
                                    'name': 'Arm2.R',
                                    'tail': 10,
                                    'tail_direction': N_X,
                                    'constraints': [
                                        (10, Y)
                                    ]
                                }
                            ]
                        }
                    ]
                }
            ]
        },
        {
            'name': 'Leg1.L',
            'head': 11,
            'head_direction': X,
            'tail': 13,
            'tail_direction': N_Z,
            'constraints': [
                (13, Y)
            ],
            'children': [
                {
                    'name': 'Leg2.L',
                    'tail': 15,
                    'tail_direction': N_Z,
                    'constraints': [
                        (15, Y)
                    ]
                }
            ]
        },
        {
            'name': 'Leg1.R',
            'head': 12,
            'head_direction': N_X,
            'tail': 14,
            'tail_direction': N_Z,
            'constraints': [
                (14, Y)
            ],
            'children': [
                {
                    'name': 'Leg2.R',
                    'tail': 16,
                    'tail_direction': N_Z,
                    'constraints': [
                        (16, Y)
                    ]
                }
            ]
        }
    ]
}

person26 = {
    'name': 'Hips',
    'head': 19,
    'tail': (18, 19),
    'tail_direction': Z,
    'constraints': [
        19,
        (11, X), (12, N_X),
        (18, X, Y)
    ],
    'children': [
        {
            'name': 'Chest',
            'tail': 18,
            'tail_direction': Z,
            'constraints': [
                (18, Y),
                (5, Y, X), (6, Y, N_X)
            ],
            'children': [
                {
                    'name': 'Head',
                    'tail': 17,
                    'tail_direction': Z,
                    'rotation': [
                        17, 18,
                        (17, Y),
                        (0, Y, Z), (1, Y, Z), (2, Y, Z)
                    ]
                },
                {
                    'name': 'Shoulder.L',
                    'tail': 5,
                    'tail_direction': X,
                    'constraints': [
                        (5, Y)
                    ],
                    'children': [
                        {
                            'name': 'Arm1.L',
                            'tail': 7,
                            'tail_direction': X,
                            'constraints': [
                                (7, Y)
                            ],
                            'children': [
                                {
                                    'name': 'Arm2.L',
                                    'tail': 9,
                                    'tail_direction': X,
                                    'constraints': [
                                        (9, Y)
                                    ]
                                }
                            ]
                        }
                    ]
                },
                {
                    'name': 'Shoulder.R',
                    'tail': 6,
                    'tail_direction': N_X,
                    'constraints': [
                        (6, Y)
                    ],
                    'children': [
                        {
                            'name': 'Arm1.R',
                            'tail': 8,
                            'tail_direction': N_X,
                            'constraints': [
                                (8, Y)
                            ],
                            'children': [
                                {
                                    'name': 'Arm2.R',
                                    'tail': 10,
                                    'tail_direction': N_X,
                                    'constraints': [
                                        (10, Y)
                                    ]
                                }
                            ]
                        }
                    ]
                }
            ]
        },
        {
            'name': 'Leg1.L',
            'head': 11,
            'head_direction': X,
            'tail': 13,
            'tail_direction': N_Z,
            'constraints': [
                (13, Y)
            ],
            'children': [
                {
                    'name': 'Leg2.L',
                    'tail': 15,
                    'tail_direction': N_Z,
                    'constraints': [
                        (15, Y),
                        (20, Y, N_Z), (22, Y, N_Z)
                    ],
                    'children': [
                        {
                            'name': 'Foot.L',
                            'tail': (20, 22),
                            'tail_direction': N_Y,
                            'rotation': [
                                15,
                                (20, Y), (22, Y),
                                (24, Y, N_Z), (20, Y, X), (22, Y, N_X)
                            ]
                        }
                    ]
                }
            ]
        },
        {
            'name': 'Leg1.R',
            'head': 12,
            'head_direction': N_X,
            'tail': 14,
            'tail_direction': N_Z,
            'constraints': [
                (14, Y)
            ],
            'children': [
                {
                    'name': 'Leg2.R',
                    'tail': 16,
                    'tail_direction': N_Z,
                    'constraints': [
                        (16, Y),
                        (21, Y, N_Z), (23, Y, N_Z)
                    ],
                    'children': [
                        {
                            'name': 'Foot.R',
                            'tail': (21, 23),
                            'tail_direction': N_Y,
                            'rotation': [
                                16,
                                (21, Y), (23, Y),
                                (25, Y, N_Z), (21, Y, N_X), (23, Y, X)
                            ]
                        }
                    ]
                }
            ]
        }
    ]
}
