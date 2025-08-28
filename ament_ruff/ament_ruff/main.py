#!/usr/bin/env python3

# Copyright 2025 Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Southwest Research Institute® (SwRI®) nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
import pathlib
import subprocess
import sys
import time
from xml.sax.saxutils import escape, quoteattr

import argcomplete
from unidiff import PatchSet


def get_xunit_content(report, name, duration, checked_files):
    test_count = sum(max(len(r), 1) for r in report.values())
    error_count = sum(len(r) for r in report.values())
    xml = f"""<?xml version='1.0' encoding='UTF-8'?>
<testsuite
  name='{name}'
  tests='{test_count}'
  errors='0'
  failures='{error_count}'
  time='{duration:.3f}'
>
"""
    for filename in sorted(report.keys()):
        chunks = report[filename]
        if chunks:
            for chunk in chunks:
                xml += f"""  <testcase
    name={quoteattr(f'{filename}:{chunk.source_start}')}
    classname='{name}'
  >
      <failure message={quoteattr(str(chunk))}></failure>
  </testcase>
"""
        else:
            checked_files_dict = {
                'checked_files': escape(''.join([f'\n* {r}' for r in sorted(checked_files)]))
            }
            xml += f"""  <testcase
    name={quoteattr(f'{filename}:{chunk.source_start}')}
    classname="{name}"/>
  <system-out>Checked files:{checked_files_dict}</system-out>
"""
    xml += '</testsuite>\n'
    return xml


def main(argv=sys.argv):
    parser = argparse.ArgumentParser(
        description='Check or format python code using ruff',
    )
    parser.add_argument(
        'paths',
        nargs='*',
        default=[pathlib.Path(os.curdir)],
        help='The files or directories to check',
        type=pathlib.Path,
    )
    parser.add_argument(
        '--config',
        metavar='path',
        default=os.path.join(os.path.dirname(__file__), 'configuration', 'ruff.toml'),
        dest='config_file',
        help='The path to a pyproject.toml or ruff.toml config file.',
    )
    parser.add_argument(
        '--reformat',
        action='store_true',
        help='Reformat the files in place',
    )
    parser.add_argument(
        '--xunit-file',
        help='Generate a xunit compliant XML file',
    )

    argcomplete.autocomplete(parser)
    args = parser.parse_args(sys.argv[1:])
    if args.config_file is not None and not os.path.exists(args.config_file):
        print(f'Could not find the config file {args.config_file}', file=sys.stderr)
        return 1

    if args.xunit_file:
        start_time = time.time()
    ruff_argv = []
    if args.config_file is not None:
        ruff_argv.extend(['--config', args.config_file])

    ruff_argv.extend(args.paths)

    out_check = subprocess.run(['ruff', 'check', *ruff_argv, '--diff'], capture_output=True, text=True)
    out_format = subprocess.run(['ruff', 'format', *ruff_argv, '--diff'], capture_output=True, text=True)
    ruff_find_files = subprocess.run(
        ['ruff', 'check', *ruff_argv, '--show-files'], capture_output=True, text=True
    )
    patches = PatchSet(out_check.stdout)
    patches += PatchSet(out_format.stdout)
    checked_files = [str(p) for p in ruff_find_files.stdout]
    changed_files = []
    report = {}
    for patch in patches:
        filename = patch.source_file
        changed_files.append(filename)
        report[filename] = patch

    file_count = sum(1 if report[k] else 0 for k in report)
    replacement_count = sum(len(r) for r in report.values())

    if args.reformat:
        res_format = subprocess.run(['ruff', 'format', *ruff_argv], capture_output=True)
        res_check = subprocess.run(['ruff', 'check', *ruff_argv, '--fix'], capture_output=True)
        success = (res_check.returncode == 0) and (res_format.returncode == 0)
        if success:
            print(
                f'Fixed and formatted {file_count} files with {replacement_count}'
                ' code style divergences'
            )
            return 0
        else:
            print('Reformat failed! Some may not be automatically fixable.')
            print(f'Check output:\n{res_check.stderr.decode() + res_check.stdout.decode()}')
            print(f'\nFormat output: {res_format.stderr.decode() + res_format.stdout.decode()}')
            return 1

    if not file_count:
        print('No problems found')
        code = 0
    else:
        current_dir = os.path.curdir
        print(f'{file_count} files with {replacement_count} code style divergences')
        for changed_file in report:
            report[changed_file]
            print(
                f'\t{os.path.relpath(changed_file, current_dir)} has {len(report[changed_file])}'
                ' code style divergences'
            )
        code = 1

    if args.xunit_file:
        folder_name = os.path.basename(os.path.dirname(args.xunit_file))
        file_name = os.path.basename(args.xunit_file)
        suffix = '.xml'
        if file_name.endswith(suffix):
            file_name = file_name.split(suffix)[0]
        test_name = f'{folder_name}.{file_name}'
        xml = get_xunit_content(report, test_name, time.time() - start_time, checked_files)
        path = os.path.dirname(os.path.abspath(args.xunit_file))
        if not os.path.exists(path):
            os.makedirs(path)
        with open(args.xunit_file, 'w') as f:
            f.write(xml)
    return code


if __name__ == '__main__':
    exit(main())
