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
import json
import os
import re
import sys

# String indicating the line when the summary report starts
report_start_str = 'The following errors were found'

# a list of known MC/DC checker codes
checker_codes = [
                 'clang_parse_failed',
                 'failed_to_create_bdd',
                 'invalid_operator_nesting'
                 'unexpected_node',
                 'bdd_is_not_tree_like',
                 'bdd_is_not_tree_like_and_has_too_many_nodes'
                ]

def format_result(rule_id, level, message, location_uri, line_no, column_no):
    """
    Convert MC/DC checker summary output to SARIF format.

    :rule_id: Identifier of the rule that was evaluated to produce the result
    :level: Severity level
    :message: A string describing the result
    :location_uri: Location in code where the tool detects a result
    :line_no: Line number
    :column_no: Column number
    """
    output = {}
    output['ruleId'] = rule_id
    output['level'] = level
    output['message'] = {'text': message}
    physical_location =  {'physicalLocation': {
                            'artifactLocation': {
                              'uri': location_uri,
                             },
                           }
                         }

    if line_no and column_no and line_no.isdigit() and column_no.isdigit():
        physical_location['region'] = {
                                      'startLine': line_no,
                                      'startColumn': column_no
                                     }

    locations = []
    locations.append(physical_location)
    output['locations'] = locations
    return output

def convert_summary_to_sarif_output(data):
    """
    Convert MC/DC checker summary output to SARIF format.

    :data: Lines to convert to SARIF format
    """
    results = []

    for code in checker_codes:
        # if errors exist for a particular error code
        if code in data.keys() and len(lines := data[code]) > 0:
            # produce one result (in sarif terms) per line
            for line in lines:
                # All error outputs start with 'file '
                if line.startswith('file '):
                    rule_id = code
                    level = 'error'
                    message = f'{code}'
                    uri = ''
                    line_no = None
                    column_no = None
                    # run regex to get filename, line no and column no
                    # line no and column no are optional so they are placed in
                    # a non-capture group (:?) in the regex search str
                    pattern = 'file (.+?)(?: in line ([0-9]+) column ([0-9]+))?$'
                    m = re.search(pattern, line)
                    if m:
                        uri = m.group(1)
                        line_no = m.group(2)
                        column_no = m.group(3)
                    result = format_result(rule_id,
                                           level,
                                           message,
                                           uri,
                                           line_no,
                                           column_no)
                    results.append(result)
                # if it's a solution, it should be for the previous error
                # so append it to the message field of previous line item
                elif line.startswith ('Found solution'):
                    results[-1]['message']['text'] += f'. {line}'

    return results


def convert_pre_summary_to_sarif_output(lines):
    """
    Convert MC/DC checker pre-summary output to SARIF format.

    Pre-summary output refers to all output lines produced by the checker
    before the summary.

    :data: Lines to convert to SARIF format
    """

    results = []
    for line in lines:
        line_no = None
        column_no = None
        l = line.lstrip()
        if l.startswith('ERROR') and 'Clang' in l:
            pattern = 'file (.+)'
            m = re.search(pattern, line)
            if m:
                uri = m.group(1)
                rule_id = 'clang_preprocessor_error'
                level = 'error'
                message = l
                result = format_result(rule_id,
                                       level,
                                       message,
                                       uri,
                                       line_no,
                                       column_no)
                results.append(result)

        else:
            pattern = 'file (.+?)(?: at line ([0-9]+), column ([0-9]+))'
            m = re.search(pattern, line)
            if m:
                uri = m.group(1)
                line_no = m.group(2)
                column_no = m.group(3)

                rule_id = 'non-tree-like_decision'
                level = 'error'
                message = l
                result = format_result(rule_id,
                                       level,
                                       message,
                                       uri,
                                       line_no,
                                       column_no)
                results.append(result)

    return results

def parse_summary_for_error(lines, error):
    """
    Parse and filter the summary output to contain only lines related to the
    specified MC/DC checker error code

    :lines: Lines from the summary output
    :error: Error code to look for
    """

    output = {error: []}
    start = False
    for line in lines:
        if start:
            l = line.lstrip()
            # valid error pointing to a file
            if l.startswith('file '):
                output[error].append(l)
            # if line is another error code, exit
            elif any(c in l for c in checker_codes):
                break
            # other output produced for current error code
            # e.g. Found solutions
            else:
                output[error].append(l)
        # found start of section for this error code
        elif error in line:
            start = True
    return output

def main():
    """
    Main parse function that reads the MC/DC checker output text and converts it
    to SARIF format

    :file_path: Path to raw MC/DC checker output file
    :parse_all: True to parse all output. False to parse only the summary
    :file_output: Path to raw MC/DC checker output file
    """

    parser = argparse.ArgumentParser(description='MC/DC checker output parser')
    parser.add_argument(
        'file',
        type=str,
        nargs='?',
        default=None,
        help='Path to MC/DC checker output text file',
    )
    parser.add_argument(
        "-a",
        "--all",
        action="store_true",
        required=False,
        help="Parse all MC/DC checker output, including output before the summary",
    )
    parser.add_argument(
        "-o",
        "--out",
        type=str,
        required=False,
        help="Path to save the SARIF output to. Prints to console if not specified",
    )
    parser.add_argument(
        "-r",
        "--results-only",
        action="store_true",
        required=False,
        help="Print only the results section of the SARIF output.",
    )

    args = parser.parse_args()
    if not args.file:
        parser.print_usage()
        sys.exit(1)

    file_path = args.file
    parse_all = args.all
    file_output = args.out
    results_only = args.results_only

    # parse the raw text mcdc_checker output
    with open(file_path) as f:

        # parse file into lines
        lines = f.read().splitlines();
        line_idx = 0
        # find the line when report summary starts
        while line_idx < len(lines):
            line = lines[line_idx]
            line_idx += 1
            if report_start_str in line:
                break

        # if report summary start line is found
        if line_idx < len(lines):
            # get all output before summary eport
            pre_summary_lines = lines[:line_idx-1]

            # get summary report output
            summary_lines = lines[line_idx:]

            data = {}
            results = []

            # write parsed data to sarif output
            if parse_all:
                results = convert_pre_summary_to_sarif_output(pre_summary_lines)

            # parse the file for all error types
            for code in checker_codes:
                out = parse_summary_for_error(summary_lines, code)
                data.update(out)

            # write parsed data to sarif output
            summary_results = convert_summary_to_sarif_output(data)
            results.extend(summary_results)

            output = {}
            if results_only:
                output = results
            else:
                # Output is generated based on this version spec
                output['version'] = '2.1.0'
                output['runs'] = [{
                                   'tool': {
                                     'driver': {
                                       'name': 'mcdc_checker'
                                     }
                                   },
                                   'results': results
                                 }]

            out = json.dumps(output, indent = 2)
            if not file_output:
                print(out)
            else:
                out_f = open(file_output, 'w')
                out_f.write(out)
                out_f.close()


if __name__ == '__main__':
    main()
