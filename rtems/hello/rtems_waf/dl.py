#
# RTEMS Project (https://www.rtems.org/)
#
# Copyright (c) 2018 Chris Johns <chrisj@rtems.org>. All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

def _syms_rule(tsk):
    '''
    A rule handler so 'no_errcheck_out' can be set. This avoids the
    erronous duplicate output error from waf (2.0.14 and later).
    '''
    setattr(tsk, 'no_errcheck_out', True)
    src = tsk.inputs[0].abspath()
    tgt = tsk.outputs[0].abspath()
    cmd = '%s -e -C %s -c "%s" -o %s %s' % (' '.join(tsk.env.RTEMS_SYMS),
                                            ' '.join(tsk.env.CC),
                                            ' '.join(tsk.env.CFLAGS),
                                            tgt,
                                            src)
    return tsk.exec_command(cmd)

def syms(ctx, target, source):
    '''
    Create a symbols object file from a base kernel image. The object
    can be linked to the file executable providing it with a symbol
    table.

    The created object file is read and available in a 'use' attribute
    of the 'cprogram' build.

    :param ctx: Waf build context
    :param target: The target object file to create and read
    :param source: The kernel base image to generate the symbol table of
    '''
    tgt = ctx.path.find_or_declare(target)
    ctx(rule = _syms_rule,
        target = tgt,
        source = source,
        color = 'CYAN')
    ctx.read_object(tgt)

def _strip_rule(tsk):
    '''
    A rule handler so 'no_errcheck_out' can be set. We need this because
    'ranlib' takes only a single argument, the archive is rewritten so it
    will appear in 2 outputs.
    '''
    setattr(tsk, 'no_errcheck_out', True)
    src = tsk.inputs[0].abspath()
    tgt = tsk.outputs[0].abspath()
    cmd = '%s -d -o %s %s' % (' '.join(tsk.env.STRIP), tgt, src)
    return tsk.exec_command(cmd)

def strip_debug_info(ctx, *k, **kw):
    '''
    Strip the source object file or archive of debug information
    creating a new archive in the build directory.

    :param ctx: Waf build context
    :param target: The stripped target archive or object file
    :param source: The source target or acthive file to strip
    '''
    if 'source' not in kw:
        ctx.fatal('No source in strip')
    if 'target' not in kw:
        ctx.fatal('No target in strip')
    source = kw['source']
    target = kw['target']
    if 'name' in kw:
        name = kw['name']
    else:
        if not isinstance(source, str):
            ctx.fatal('No name and source is not a path')
        name = 'strip-%s' % (os.path.basename(source))
    print(type(source), str(source), target)
    ctx(rule = _strip_rule,
        name = name,
        target = target,
        source = source,
        color = 'CYAN')

def _ranlib_rule(tsk):
    '''
    A rule handler so 'no_errcheck_out' can be set. We need this because
    'ranlib' takes only a single argument, the archive is rewritten so it
    will appear in 2 outputs.
    '''
    setattr(tsk, 'no_errcheck_out', True)
    tgt = tsk.inputs[0].abspath()
    cmd = '%s -t %s' % (' '.join(tsk.env.RANLIB), tgt)
    return tsk.exec_command(cmd)

def ranlib(ctx, lib):
    ctx(rule = _ranlib_rule,
        name = 'ranlib-%s' % (lib),
        source = lib)
