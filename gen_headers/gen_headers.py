"""
Copyright (C) 2025 Ian Sloat
Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>.

Generates headers that can be used to access internal memory structures
from blender.
"""

import argparse
import json
import pickle
import bisect
import os
import code_analysis
import git_tools
import output


def get_args():
    parser = argparse.ArgumentParser(
        prog=os.path.basename(__file__),
        description="Generates C style headers for multi-version compatability with Blender's DNA structures",
    )
    parser.add_argument("--ignore-cache",  action="store_true",
                        help="ignore cached syntax trees")
    parser.add_argument("--min-ver", default=None,
                        help="(optional) minimum blender version")
    parser.add_argument("--max-ver", default=None,
                        help="(optional) maximum blender version")
    return parser.parse_args()


def get_cache():
    target_files = []
    for file in os.listdir("syntax_trees"):
        if not file.endswith(".pickle") and not file.startswith("dna_ast_"):
            continue
        v = file.removesuffix(".pickle").split("_")
        try:
            v = (int(v[2]), int(v[3]), int(v[4]))
        except ValueError:
            continue
        target_files.append((v, os.path.join("syntax_trees", file)))
    target_files.sort(key=lambda a: a[0])
    return target_files


def main():
    args = get_args()
    if args.ignore_cache:
        print("WARNING: Rebuilding the cache can take a long time!")
        prompt = input("Do you wish to continue? [y]: ")
        if prompt and prompt.lower() != 'y':
            return
    print("Initializing blender repository...")
    git_tools.init_repo()
    versions = git_tools.available_versions()
    dna_dir = 'blender/source/blender/makesdna'
    os.makedirs("syntax_trees", exist_ok=True)
    if args.ignore_cache:
        missing = versions
    else:
        missing = [v for v in versions if v not in [f[0] for f in get_cache()]]
    for v in missing:
        print(f"Checking out blender version {v[0]}.{v[1]}.{v[2]}")
        git_tools.checkout_version(v)
        print(f"Generating abstract syntax tree for makesdna structures...")
        ast = code_analysis.SyntaxTree()
        for file in os.listdir(dna_dir):
            if not file.endswith((".hpp", ".h")):
                continue
            ast.merge(code_analysis.SyntaxTree.from_file(
                os.path.join(dna_dir, file)).get_fixed_tree())
        output_file = os.path.join(
            "syntax_trees", f"dna_ast_{v[0]}_{v[1]}_{v[2]}.pickle")
        with open(output_file, 'wb') as f:
            pickle.dump(ast, f)
    history_file = os.path.join("syntax_trees", "dna_ast_all.pickle")
    if not os.path.exists(history_file) or missing or args.ignore_cache:
        target_files = get_cache()
        versioned_ast = code_analysis.VersionedSyntaxTree()
        for v, path in target_files:
            print(f"Merging AST for blender version {v[0]}.{v[1]}.{v[2]}...")
            with open(path, "rb") as f:
                versioned_ast.add_tree(pickle.load(f), v)
        if None in versioned_ast.master_tree.values():
            raise ValueError("Something nasty has happened")
        with open(history_file, 'wb') as f:
            pickle.dump(versioned_ast, f)
    else:
        with open(history_file, 'rb') as f:
            versioned_ast = pickle.load(f)
    print("Writing headers...")
    versions = versioned_ast.get_versions()
    if args.min_ver:
        min_ver = []
        for i, v in enumerate(args.min_ver.split(".")):
            if i < 3:
                min_ver.append(int(v))
            else:
                break
        if len(min_ver) < 3:
            min_ver += [0] * (3 - len(min_ver))
        min_ver = tuple(min_ver)
        min_ver_idx = max(0, bisect.bisect(versions, min_ver) - 1)
        min_ver = versions[min_ver_idx]
    else:
        min_ver_idx = 0
        min_ver = versions[0]
    if args.max_ver:
        max_ver = []
        for i, v in enumerate(args.max_ver.split(".")):
            if i < 3:
                max_ver.append(int(v))
            else:
                break
        if len(max_ver) < 3:
            max_ver += [0] * (3 - len(max_ver))
        max_ver = tuple(max_ver)
        max_ver_idx = max(0, bisect.bisect(versions, max_ver) - 1)
        max_ver = versions[max_ver_idx]
    else:
        max_ver_idx = len(versions) - 1
        max_ver = versions[-1]
    versions = versions[min_ver_idx:max_ver_idx + 1]
    os.makedirs("output", exist_ok=True)
    header_prefix = "makesdna_"
    for v in versions:
        ast = versioned_ast.get_tree(v)
        with open(os.path.join("output", f"{header_prefix}{v[0]}_{v[1]}_{v[2]}.h"), "w") as f:
            f.write(output.gen_header(ast, header_prefix, min_ver))
    macro_file = "makesdna_macros.h"
    with open(os.path.join("output", macro_file), "w") as f:
        f.write(output.generate_macros(
            versioned_ast, macro_file, min_ver, max_ver))
    types_file = "makesdna_types.hpp"
    with open(os.path.join("output", types_file), "w") as f:
        f.write(output.generate_types_hpp(versioned_ast, types_file,
                min_ver, max_ver, macro_file, header_prefix))


if __name__ == "__main__":
    main()
