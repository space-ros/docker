#
# RTEMS Project (https://www.rtems.org/)
#
# Copyright (c) 2017 Chris Johns <chrisj@rtems.org>. All rights reserved.
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

def join(*paths):
    path = ''
    for p in paths:
        path = os.path.join(path, str(p))
    return path

def copy(ctx, name, root, target, source):
    '''Copy a file from the source to the target.'''
    if isinstance(target, str):
        target = ctx.path.make_node(target)
    #print('copy: name=%s source=%s target=%s' % (name, source, target))
    ctx(rule = 'cp ${SRC} ${TGT}',
        name = name,
        source = source,
        target = target)

def tar(ctx, name, root, target, source, depends_on):
    #print('tar: name=%s root=%s target=%r source=%r' % (name, root, target, source))
    ctx(rule = 'tar -C %s -cf ${TGT} .' % (root),
        name = name,
        target = target,
        source = source,
        root = join(ctx.path.get_bld(), root),
        depends_on = depends_on,
        color = 'CYAN')

def bin2c(ctx, name, target, source):
    ctx(rule = '${RTEMS_BIN2C} ${SRC} ${TGT}',
        name = name,
        target = target,
        source = source,
        color = 'PINK')

def build(ctx, name, root, files):
    """The files are truples of the name, source and target files to put in the tar
       file. The truple is (name, src, dst). The src is the absolute path to the
       source and the dst is the path on the target.

       The tar file will contain the files defined by the dst paths. These are
       copied into the build path under the tar file's root path. Make sure the
       desination paths are relative to the root of the tar file.

       For example:
          import rtems_waf.rootfs as rtems_rootfs
          tar_files = [('shell-init', ''shell-init', 'shell-init'),
                       ('rc-conf', 'rc.conf', 'etc/rc.conf')]
          rtems_rootfs.build(ctx, 'fs-root', 'rootfs', tar_files)
    """
    #
    # The files must be a list of tuples.
    #
    if not isinstance(files, list):
        ctx.fatal('rootfs build files is not a list')

    root_abspath = join(ctx.path.get_bld().abspath(), root)

    for f in files:
        #
        # Check each item in the list is a tuple with 3 elements.
        #
        if not isinstance(f, tuple):
            ctx.fatal('rootfs build file is not a tuple')
        if len(f) != 3:
            ctx.fatal('rootfs build file tuple has 3 items (name, src, dst): %s' % (str(f)))
        #
        # Copy the file as a build task. The file is copied to the tar file's
        # root.
        #
        #print(']]', ctx.path.make_node(join(root, f[2])).get_bld())
        if isinstance(f[1], str):
            source = ctx.path.make_node(f[1]).get_src()
        else:
            source = f[1]
        copy(ctx,
             name = f[0],
             root = root,
             target = ctx.path.make_node(join(root, f[2])).get_bld(),
             source = source)

    ctx.add_group()

    #
    # Tar build task.
    #
    tar(ctx,
        name = name + '-tar',
        root = join(ctx.path.get_bld(), root),
        target = name + '.tar',
        source = [join(root, f[2]) for f in files],
        depends_on = [f[0] for f in files])

    ctx.add_group()

    #
    # Binary to C build task. It converts the tar file to a C file. This uses
    # the RTEMS Tools Project's `bin2c` command.
    #
    bin2c(ctx,
          name = name,
          target = name + '-tar.c',
          source = name + '.tar')

    ctx.add_group()

    ctx.objects(features = 'c',
                target = name + '-obj',
                source = name + '-tar.c')

def build_from_src_root(ctx, name, root):
    root_path = ctx.path.make_node(root)
    if not root_path.exists():
        ctx.fatal('tar root not found: %s' % (root_path))
    sources = [s.path_from(root_path) for s in root_path.ant_glob('**')]
    build(ctx, name, root, [('%s-%s' % (name, os.path.basename(s)),
                             join(root, s), s) for s in sources])
