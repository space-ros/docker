#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time


def get_file_groups(paths, extensions, exclude_patterns):
    excludes = []
    for exclude_pattern in exclude_patterns:
        excludes.extend(glob.glob(exclude_pattern))
    excludes = {os.path.realpath(x) for x in excludes}

    # dict mapping root path to files
    groups = {}
    for path in paths:
        if os.path.isdir(path):
            for dirpath, dirnames, filenames in os.walk(path):
                if 'AMENT_IGNORE' in dirnames + filenames:
                    dirnames[:] = []
                    continue
                # ignore folder starting with . or _
                dirnames[:] = [d for d in dirnames if d[0] not in ['.', '_']]
                dirnames.sort()

                # select files by extension
                for filename in sorted(filenames):
                    _, ext = os.path.splitext(filename)
                    if ext in ('.%s' % e for e in extensions):
                        filepath = os.path.join(dirpath, filename)
                        if os.path.realpath(filepath) not in excludes:
                            append_file_to_group(groups, filepath)

        if os.path.isfile(path):
            if os.path.realpath(path) not in excludes:
                append_file_to_group(groups, path)

    return groups


def append_file_to_group(groups, path):
    path = os.path.abspath(path)

    root = ''

    # try to determine root from path
    base_path = os.path.dirname(path)
    # find longest subpath which ends with one of the following subfolder names
    subfolder_names = ['include', 'src', 'test']
    matches = [
        re.search(
            '^(.+%s%s)%s' %
            (re.escape(os.sep), re.escape(subfolder_name), re.escape(os.sep)), path)
        for subfolder_name in subfolder_names]
    match_groups = [match.group(1) for match in matches if match]
    if match_groups:
        match_groups = [{'group_len': len(x), 'group': x} for x in match_groups]
        sorted_groups = sorted(match_groups, key=lambda k: k['group_len'])
        base_path = sorted_groups[-1]['group']
        root = base_path

    # try to find repository root
    repo_root = None
    p = path
    while p and repo_root is None:
        # abort if root is reached
        if os.path.dirname(p) == p:
            break
        p = os.path.dirname(p)
        for marker in ['.git', '.hg', '.svn']:
            if os.path.exists(os.path.join(p, marker)):
                repo_root = p
                break

    # compute relative --root argument
    if repo_root and repo_root > base_path:
        root = os.path.relpath(base_path, repo_root)

    # add the path to the appropriate group
    if root not in groups:
        groups[root] = []
    groups[root].append(path)


def write_sarif_file(output_filename, input_file_path, verbose):
    """Invoke MC/DC checker output parser and output result to sarif."""

    # assume the mcdc_checker_output_parser.py is in the same dir as this script
    dir_path = os.path.dirname(os.path.realpath(__file__))
    parser_script = os.path.join(dir_path, 'mcdc_checker_output_parser.py')

    # loop through all mcdc_checker output files, parse and merge them to
    # one single sarif output
    all_results = []
    for filename in os.listdir(input_file_path):
        # this tells the parser to parse all output and generate sarif for the
        # results section only. We will combine all results and put them into
        # one 'run' in the sarif output
        arguments = ['python3', parser_script, '-a', '-r']
        cmd_output = ""
        try:
            arguments.append(os.path.join(input_file_path, filename))
            if verbose:
                print(' '.join(arguments))
            p = subprocess.Popen(arguments,
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT)
            cmd_output = p.communicate()[0]
        except subprocess.CalledProcessError as e:
            print("The invocation of 'mcdc_checker_output_parser' failed with error code %d: %s" %
                  (e.returncode, e), file=sys.stderr)
            return False

        out = cmd_output.decode('utf-8')
        results = json.loads(out)
        all_results.extend(results)

    # generate sarif output with all results combined
    output = {}
    output['version'] = '2.1.0'
    output['runs'] = [{
                       'tool': {
                         'driver': {
                           'name': 'mcdc_checker'
                         }
                       },
                       'results': all_results
                     }]

    out = json.dumps(output, indent = 2)
    f = open(output_filename, 'w')
    f.write(out)
    f.close()
    return True


def find_executable(file_name, additional_paths=None):
    path = None
    if additional_paths:
        path = os.getenv('PATH', os.defpath)
        path += os.path.pathsep + os.path.pathsep.join(additional_paths)
    return shutil.which(file_name, path=path)


def invoke_mcdc_checker(arguments, output_path, file_path, verbose):
    """Invoke MC/DC checker and write output to file."""
    try:
        if verbose:
            print(' '.join(arguments))
        p = subprocess.Popen(arguments,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
        cmd_output = p.communicate()[0]
    except subprocess.CalledProcessError as e:
        print("The invocation of 'mcdc_checker' failed with error code %d: %s" %
              (e.returncode, e), file=sys.stderr)
        return False

    output = cmd_output.decode('utf-8')

    # save output to tmp file
    # this will be read by mcdc_checker_output_parser later if user indicates
    # to output to sarif
    output_file = ""
    filename = file_path.replace(os.sep, '_')
    if filename[0] == '_':
        filename = filename[1:]
    output_file = os.path.join(output_path, filename + ".mcdc")

    f = open(output_file, 'w')
    f.write(output)
    f.close()
    return True


def main(argv=sys.argv[1:]):
    rc = 0
    extensions = ['c', 'cc', 'cpp', 'cxx']

    # Define and parse the command-line options
    parser = argparse.ArgumentParser(
        description='Run the MC/DC checker tool.')
    parser.add_argument(
        'paths',
        nargs='*',
        default=[os.curdir],
        help='Files and/or directories to be checked. Directories are searched recursively for '
             'files ending in one of %s.' %
             ', '.join(["'.%s'" % e for e in extensions]))
    parser.add_argument(
        '--include_dirs',
        nargs='*',
        help='Include directories for C/C++ files being checked.'
             "Each directory is passed to cobra as '-I<include_dir>'")
    parser.add_argument(
        '--exclude', default=[],
        nargs='*',
        help='Exclude C/C++ files from being checked.')
    parser.add_argument(
        '--compile_cmds',
        help='The compile_commands.json file from which to gather preprocessor directives.')
    parser.add_argument(
        '--sarif_file',
        help='Generate a SARIF file')
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Display verbose output')

    args = parser.parse_args(argv)

    target_binary = 'mcdc_checker'
    mcdc_checker_bin = find_executable(target_binary)
    if not mcdc_checker_bin:
        print(f"Error: Could not find the '{target_binary}' executable", file=sys.stderr)
        return 1

    groups = get_file_groups(args.paths, extensions, args.exclude)
    if not groups:
        print('No files found', file=sys.stderr)
        return 1

    cmd = [mcdc_checker_bin]

    # Get the preprocessor options to use for each file from the
    # input compile_commands.json file
    options_map = {}
    if args.compile_cmds:
        f = open(args.compile_cmds)
        compile_data = json.load(f)

        for item in compile_data:
            compile_options = item['command'].split()

            preprocessor_options = []
            options = iter(compile_options)
            for option in options:
                if option in ['-D', '-I']:
                    preprocessor_options.extend([option, options.__next__()])
                elif option == '-isystem':
                    preprocessor_options.extend(['-I' + options.__next__()])
                elif option.startswith(('-D', '-I', '-U')):
                    preprocessor_options.extend([option])

            options_map[item['file']] = {
                'directory': item['directory'],
                'options': preprocessor_options
            }

    # create tmp dir for storing mcdc_chekcer output
    mcdc_output_path = 'mcdc_raw_output'
    if os.path.exists(mcdc_output_path):
        shutil.rmtree(mcdc_output_path)
    os.mkdir(mcdc_output_path)

    if args.verbose:
        print('Invoking mcdc_checker. This may take a while')

    # For each group of files
    # Run the mcdc_checker
    success = True
    for group_name in sorted(groups.keys()):
        files_in_group = groups[group_name]

        # If a compile_commands.json is provided, process each source file
        # separately, with its associated preprocessor directives
        if args.compile_cmds:
            for filename in files_in_group:
                if filename in options_map and options_map[filename]['options']:
                    arguments = cmd + options_map[filename]['options'] + [filename]
                else:
                    arguments = cmd + [filename]

                success = invoke_mcdc_checker(arguments, mcdc_output_path, filename, args.verbose)
                if not success:
                    rc = 1
                    print(f'There were errors running mcdc_checker on {filename}.')
        # Otherwise, run mcdc_checker on this group of files
        else:
            includes = []
            for include_dir in (args.include_dirs or []):
                includes.extend(['-I' + include_dir])

            # mcdc_checker takes either path to a single directory or file but
            # but not multiple files so run it for each file in the group
            for filename in files_in_group:
                arguments = cmd + includes
                arguments.extend([filename])
                success = invoke_mcdc_checker(arguments, mcdc_output_path, filename,  args.verbose)
                if not success:
                    rc = 1
                    print(f'There were errors running mcdc_checker on {filename}.')

    if args.verbose:
        print('Done running mcdc_checker.')

    # run mcdc_checker_output_parser to convert to sarif format
    if args.sarif_file:
        if args.verbose:
            print('Converting output to SARIF format.')
        success = write_sarif_file(args.sarif_file, mcdc_output_path, args.verbose)
        if not success:
            rc = 1
            print('There were errors running mcdc_checker_output_parser.py.')
        if args.verbose:
            print(f'SARIF file saved to {args.sarif_file}.')

    return rc


if __name__ == '__main__':
    sys.exit(main())
