
# Copyright 2012-2016 Chris Johns (chrisj@rtems.org)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

#
# RTEMS support for applications.
#

import copy
import os
import os.path
from . import pkgconfig
import re
import subprocess
import sys

rtems_default_version = None
rtems_filters = None
rtems_long_commands = False

windows = os.name == 'nt' or sys.platform in ['msys', 'cygwin']

def options(opt):
    opt.add_option('--rtems',
                   default = None,
                   dest = 'rtems_path',
                   help = 'Path to an installed RTEMS (defaults to prefix).')
    opt.add_option('--rtems-tools',
                   default = None,
                   dest = 'rtems_tools',
                   help = 'Path to RTEMS tools (defaults to path to installed RTEMS).')
    opt.add_option('--rtems-version',
                   default = None,
                   dest = 'rtems_version',
                  help = 'RTEMS version (default is derived from prefix).')
    opt.add_option('--rtems-archs',
                   default = 'all',
                   dest = 'rtems_archs',
                   help = 'List of RTEMS architectures to build.')
    opt.add_option('--rtems-bsps',
                   default = 'all',
                   dest = 'rtems_bsps',
                   help = 'List of BSPs to build.')
    opt.add_option('--show-commands',
                   action = 'store_true',
                   default = False,
                   dest = 'show_commands',
                   help = 'Print the commands as strings.')

def init(ctx, filters = None, version = None, long_commands = False, bsp_init = None):
    global rtems_filters
    global rtems_default_version
    global rtems_long_commands

    #
    # Set the RTEMS filter to the context.
    #
    rtems_filters = filters

    #
    # Set the default version, can be overridden.
    #
    rtems_default_version = version

    #
    # Set the long commands option.
    #
    rtems_long_commands = long_commands

    env = None
    contexts = []
    try:
        import waflib.Options
        import waflib.ConfigSet

        #
        # Load the configuation set from the lock file.
        #
        env = waflib.ConfigSet.ConfigSet()
        env.load(waflib.Options.lockfile)

        #
        # Check the tools, architectures and bsps.
        #
        rtems_version, rtems_path, rtems_tools, archs, arch_bsps = \
            check_options(ctx,
                          env.options['prefix'],
                          env.options['rtems_tools'],
                          env.options['rtems_path'],
                          env.options['rtems_version'],
                          env.options['rtems_archs'],
                          env.options['rtems_bsps'])

        #
        # Update the contexts for all the bsps.
        #
        from waflib.Build import BuildContext, CleanContext, \
            InstallContext, UninstallContext
        for x in arch_bsps:
            for y in (BuildContext, CleanContext, InstallContext, UninstallContext):
                name = y.__name__.replace('Context','').lower()
                class context(y):
                    cmd = name + '-' + x
                    variant = x
                contexts += [context]

        #
        # Transform the command to per BSP commands.
        #
        commands = []
        for cmd in waflib.Options.commands:
            if cmd in ['build', 'clean', 'install', 'uninstall']:
                for x in arch_bsps:
                    commands += [cmd + '-' + x]
            else:
                commands += [cmd]
        waflib.Options.commands = commands
    except:
        pass

    if bsp_init:
        bsp_init(ctx, env, contexts)

def test_application(more = []):
    code =  ['#include <rtems.h>']
    code += more
    code += ['void Init(rtems_task_argument arg) { (void)arg; }']
    code += ['#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER']
    code += ['#define CONFIGURE_MAXIMUM_TASKS 1']
    code += ['#define CONFIGURE_RTEMS_INIT_TASKS_TABLE']
    code += ['#define CONFIGURE_INIT']
    code += ['#include <rtems/confdefs.h>']
    return os.linesep.join(code)

def configure(conf, bsp_configure = None):
    #
    # Check the environment for any flags.
    #
    for f in ['CC', 'CXX', 'AS', 'LD', 'AR', 'LINK_CC', 'LINK_CXX',
              'CPPFLAGS', 'CFLAGS', 'CXXFLAGS', 'ASFLAGS', 'LINKFLAGS', 'LIB'
              'WFLAGS', 'RFLAGS', 'MFLAGS', 'IFLAGS']:
        if f in os.environ:
            conf.msg('Environment variable set', f, color = 'RED')

    #
    # Handle the configurable commands options.
    #
    if conf.options.show_commands:
        show_commands = 'yes'
    else:
        show_commands = 'no'
    if rtems_long_commands and windows:
        long_commands = 'yes'
    else:
        long_commands = 'no'

    rtems_version, rtems_path, rtems_tools, archs, arch_bsps = \
        check_options(conf,
                      conf.options.prefix,
                      conf.options.rtems_tools,
                      conf.options.rtems_path,
                      conf.options.rtems_version,
                      conf.options.rtems_archs,
                      conf.options.rtems_bsps)

    if rtems_tools is None:
        conf.fatal('RTEMS tools not found.')

    _log_header(conf)

    conf.msg('RTEMS Version', rtems_version, 'YELLOW')
    conf.msg('Architectures', ', '.join(archs), 'YELLOW')

    tools = {}
    env = conf.env.derive()
    conf.env.RTEMS_ARCH_BSP_LIST = arch_bsps

    for ab in arch_bsps:
        conf.setenv(ab, env)

        conf.msg('Board Support Package (BSP)', ab, 'YELLOW')

        #
        # Show and long commands support.
        #
        conf.env.SHOW_COMMANDS = show_commands
        conf.env.LONG_COMMANDS = long_commands

        conf.msg('Show commands', show_commands)
        conf.msg('Long commands', long_commands)

        arch = _arch_from_arch_bsp(ab)
        bsp  = _bsp_from_arch_bsp(ab)

        conf.env.ARCH_BSP = '%s/%s' % (arch.split('-')[0], bsp)

        conf.env.RTEMS_PATH = rtems_path
        conf.env.RTEMS_VERSION = rtems_version
        conf.env.RTEMS_ARCH_BSP = ab
        conf.env.RTEMS_ARCH = arch.split('-')[0]
        conf.env.RTEMS_ARCH_RTEMS = arch
        conf.env.RTEMS_BSP = bsp

        tools = _find_tools(conf, arch, rtems_tools, tools)
        for t in tools[arch]:
            conf.env[t] = tools[arch][t]

        conf.load('gcc')
        conf.load('g++')
        conf.load('gas')
        conf.load('gccdeps', tooldir = os.path.dirname(__file__))

        #
        # Get the version of the tools being used.
        #
        rtems_cc = conf.env.CC[0]
        try:
            import waflib.Context
            out = conf.cmd_and_log([rtems_cc, '--version'],
                                   output = waflib.Context.STDOUT)
        except Exception as e:
            conf.fatal('CC version not found: %s' % (e.stderr))
        #
        # First line is the version
        #
        vline = out.split('\n')[0]
        conf.msg('Compiler version (%s)' % (os.path.basename(rtems_cc)),
                                            ' '.join(vline.split()[2:]))

        flags = _load_flags(conf, ab, rtems_path)

        cflags = _filter_flags('cflags', flags['CFLAGS'],
                               arch, rtems_path)
        ldflags = _filter_flags('ldflags', flags['LDFLAGS'],
                               arch, rtems_path)

        conf.env.CFLAGS    = cflags['cflags']
        conf.env.CXXFLAGS  = cflags['cflags']
        conf.env.ASFLAGS   = cflags['cflags']
        conf.env.WFLAGS    = cflags['warnings']
        conf.env.RFLAGS    = cflags['specs']
        conf.env.MFLAGS    = cflags['machines']
        conf.env.IFLAGS    = cflags['includes']
        conf.env.LINKFLAGS = cflags['cflags'] + ldflags['ldflags']
        conf.env.LIB       = flags['LIB']
        conf.env.LIBPATH   = ldflags['libpath']

        conf.env.RTRACE_WRAPPER_ST = '-W %s'

        #
        # Checks for various RTEMS features.
        #
        conf.check_cc(fragment = test_application(),
                      execute = False,
                      msg = 'Checking for a valid RTEMS BSP installation')
        load_cpuopts(conf)

        #
        # Add tweaks.
        #
        tweaks(conf, ab)

        #
        # If the user has supplied a BSP specific configure function
        # call it.
        #
        if bsp_configure:
            bsp_configure(conf, ab)

        conf.setenv('', env)

    conf.env.RTEMS_TOOLS = rtems_tools
    conf.env.ARCHS = archs
    conf.env.ARCH_BSPS = arch_bsps

    conf.env.SHOW_COMMANDS = show_commands
    conf.env.LONG_COMMANDS = long_commands

def build(bld):
    if bld.env.SHOW_COMMANDS == 'yes':
        output_command_line()
    if bld.env.LONG_COMMANDS == 'yes':
        long_command_line()

def load_cpuopts(conf):
    options = ['RTEMS_DEBUG',
               'RTEMS_MULTIPROCESSING',
               'RTEMS_NEWLIB',
               'RTEMS_POSIX_API',
               'RTEMS_SMP',
               'RTEMS_NETWORKING']
    for opt in options:
        enabled = check_cpuopt(conf, opt)
        if enabled:
            conf.env[opt] = 'Yes'
        else:
            conf.env[opt] = 'No'

def check(conf, *k, **kw):
    if 'fragment' not in kw:
        kw['fragment'] = test_application()
    conf.check(k, kw)

def check_cc(conf, *k, **kw):
    if 'fragment' not in kw:
        kw['fragment'] = test_application()
    conf.check_cc(*k, **kw)

def check_lib_path(ctx, lib, libpath = [], mandatory = True):
    lib_lib = 'lib%s.a' % (lib)
    ctx.start_msg('Library %s' % (lib_lib))
    cmd = '%s %s %s -print-file-name=%s' % (' '.join(ctx.env.CC),
                                            ' '.join(ctx.env.CFLAGS),
                                            ' '.join(['-B' + l for l in libpath]),
                                            lib_lib)
    out = ctx.cmd_and_log(cmd)
    out = os.path.normpath(out.strip())
    if out == lib_lib:
        if mandatory:
            ctx.fatal('The library %s not found' % (lib_lib))
        ctx.end_msg('not found')
    else:
        ctx.env['LIBPATH_lib%s' % (lib)] = '..' + '/..' * (ctx.path.height() - 1) + out
        ctx.end_msg('found')

def check_lib(ctx, libs):
    if not isinstance(libs, list):
        lib = [libs]
    for lib in libs:
        if 'LIBPATH_lib%s' % (lib) not in ctx.env:
            return False
    return True

def check_cpuopt(conf, opt):
    code =  ['#ifndef %s' % (opt)]
    code += ['  #error %s is not defined' % (opt)]
    code += ['#endif']
    code += ['#if %s' % (opt)]
    code += ['  /* %s is true */' % (opt)]
    code += ['#else']
    code += ['  #error %s is false' % (opt)]
    code += ['#endif']
    try:
        conf.check_cc(fragment = test_application(code),
                      execute = False,
                      msg = 'Checking for %s' % (opt))
    except conf.errors.WafError:
        return False;
    return True

def tweaks(conf, arch_bsp):
    #
    # Hack to work around NIOS2 naming.
    #
    if conf.env.RTEMS_ARCH in ['nios2']:
        conf.env.OBJCOPY_FLAGS = ['-O', 'elf32-littlenios2']
    elif conf.env.RTEMS_ARCH in ['arm']:
        conf.env.OBJCOPY_FLAGS = ['-I', 'binary', '-O', 'elf32-littlearm']
    else:
        conf.env.OBJCOPY_FLAGS = ['-O', 'elf32-' + conf.env.RTEMS_ARCH]

    #
    # Check for a i386 PC bsp.
    #
    if re.match('i386-.*-pc[3456]86', arch_bsp) is not None:
        conf.env.LINKFLAGS += ['-Wl,-Ttext,0x00100000']

    if '-ffunction-sections' in conf.env.CFLAGS:
      conf.env.LINKFLAGS += ['-Wl,--gc-sections']

def check_options(ctx, prefix, rtems_tools, rtems_path, rtems_version, rtems_archs, rtems_bsps):
    #
    # Set defaults
    #
    if rtems_version is None:
        if rtems_default_version is None:
            m = re.compile('[^0-9.]*([0-9.]+)$').match(prefix)
            if m:
                rtems_version = m.group(1)
            else:
                ctx.fatal('RTEMS version cannot derived from prefix: ' + prefix)
        else:
            rtems_version = rtems_default_version
    if rtems_path is None:
        rtems_path = prefix
    if rtems_tools is None:
        rtems_tools = rtems_path

    #
    # Check the paths are valid.
    #
    if not os.path.exists(rtems_path):
        ctx.fatal('RTEMS path not found.')
    if os.path.exists(os.path.join(rtems_path, 'lib', 'pkgconfig')):
        rtems_config = None
    elif os.path.exists(os.path.join(rtems_path, 'rtems-config')):
        rtems_config = os.path.join(rtems_path, 'rtems-config')
    else:
        ctx.fatal('RTEMS path is not valid. No lib/pkgconfig or rtems-config found.')
    rtems_share_rtems_version = os.path.join(rtems_path, 'share', 'rtems' + rtems_version)
    if not os.path.exists(os.path.join(rtems_share_rtems_version)):
        ctx.fatal('RTEMS path is not valid, "%s" not found.' % (rtems_share_rtems_version))

    #
    # We can more than one path to tools. This happens when testing different
    # versions.
    #
    rt = rtems_tools.split(',')
    tools = []
    for path in rt:
        if not os.path.exists(path):
            ctx.fatal('RTEMS tools path not found: ' + path)
        if not os.path.exists(os.path.join(path, 'bin')):
            ctx.fatal('RTEMS tools path does not contain a \'bin\' directory: ' + path)
        tools += [os.path.join(path, 'bin')]

    #
    # Filter the tools.
    #
    tools = filter(ctx, 'tools', tools)

    #
    # Match the archs requested against the ones found. If the user
    # wants all (default) set all used.
    #
    if rtems_archs == 'all':
        archs = _find_installed_archs(rtems_config, rtems_path, rtems_version)
    else:
        archs = _check_archs(rtems_config, rtems_archs, rtems_path, rtems_version)

    #
    # Filter the architectures.
    #
    archs = filter(ctx, 'archs', archs)

    #
    # We some.
    #
    if len(archs) == 0:
        ctx.fatal('Could not find any architectures')

    #
    # Get the list of valid BSPs. This process filters the architectures
    # to those referenced by the BSPs.
    #
    if rtems_bsps == 'all':
        arch_bsps = _find_installed_arch_bsps(rtems_config, rtems_path, archs, rtems_version)
    else:
        arch_bsps = _check_arch_bsps(rtems_bsps, rtems_config, rtems_path, archs, rtems_version)

    if len(arch_bsps) == 0:
        ctx.fatal('No valid arch/bsps found')

    #
    # Filter the bsps.
    #
    arch_bsps = filter(ctx, 'bsps', arch_bsps)

    return rtems_version, rtems_path, tools, archs, arch_bsps

def check_env(ctx, *env_vars):
    for v in env_vars:
        if v not in ctx.env or len(ctx.env[v]) == 0:
            return False
    return True

def check(ctx, option, setting = 'Yes'):
    if option in ctx.env:
        if isinstance(setting, bool):
            return True
        return ctx.env[option] == setting
    return False

def check_debug(ctx):
    return check(ctx, 'RTEMS_DEBUG')

def check_multiprocessing(ctx):
    return check(ctx, 'RTEMS_MULTIPROCESSING')

def check_newlib(ctx):
    return check(ctx, 'RTEMS_NEWLIB')

def check_posix(ctx):
    return check(ctx, 'RTEMS_POSIX_API')

def check_smp(ctx):
    return check(ctx, 'RTEMS_SMP')

def check_networking(ctx):
    return check(ctx, 'RTEMS_NETWORKING')

def arch(arch_bsp):
    """ Given an arch/bsp return the architecture."""
    return _arch_from_arch_bsp(arch_bsp).split('-')[0]

def bsp(arch_bsp):
    """ Given an arch/bsp return the BSP."""
    return _bsp_from_arch_bsp(arch_bsp)

def arch_bsps(ctx):
    """ Return the list of arch/bsps we are building."""
    return ctx.env.ARCH_BSPS

def arch_bsp_env(ctx, arch_bsp):
    return ctx.env_of_name(arch_bsp).derive()

def filter(ctx, filter, items):
    if rtems_filters is None:
        return items
    if type(rtems_filters) is not dict:
        ctx.fatal("Invalid RTEMS filter type, " \
                  "ie { 'tools': { 'in': [], 'out': [] }, 'arch': {}, 'bsps': {} }")
    if filter not in rtems_filters:
        return items
    items_in = []
    items_out = []
    if 'in' in rtems_filters[filter]:
        items_in = copy.copy(rtems_filters[filter]['in'])
    if 'out' in rtems_filters[filter]:
        items_out = copy.copy(rtems_filters[filter]['out'])
    filtered_items = []
    for i in items:
        item = i
        ab = '%s/%s' % (arch(item), bsp(item))
        for inre in items_in:
            if re.compile(inre).match(ab):
                items_in.remove(inre)
                filtered_items += [item]
                item = None
                break
        if item is not None:
            for outre in items_out:
                if re.compile(outre).match(ab):
                    item = None
                    break
        if item is not None:
            filtered_items += [item]
    if len(items_in) != 0:
        ctx.fatal('Following %s not found: %s' % (filter, ', '.join(items_in)))
    return sorted(filtered_items)

def arch_rtems_version(version, arch):
    """ Return the RTEMS architecture path, ie sparc-rtems4.11."""
    return '%s-rtems%s' % (arch, version)

def arch_bsp_path(version, arch_bsp):
    """ Return the BSP path."""
    return '%s/%s' % (arch_rtems_version(version, arch(arch_bsp)), bsp(arch_bsp))

def arch_bsp_include_path(version, arch_bsp):
    """ Return the BSP include path."""
    return '%s/lib/include' % (arch_bsp_path(version, arch_bsp))

def arch_bsp_lib_path(version, arch_bsp):
    """ Return the BSP library path. """
    return '%s/lib' % (arch_bsp_path(version, arch_bsp))

def library_path(library, cc, cflags):
    cmd = cc + cflags + ['-print-file-name=%s' % library]
    a = subprocess.check_output(cmd)
    lib = os.path.abspath(a.strip())
    if os.path.exists(lib):
        return os.path.dirname(lib)
    return None

def root_filesystem(bld, name, files, tar, obj):
    tar_rule = 'tar -cf ${TGT} --format=ustar -C ../.. $(echo "${SRC}" | sed -e \'s/\.\.\/\.\.\///g\')'
    if windows:
        tar_rule = 'sh -c "%s"' % (tar_rule)
    bld(name = name + '_tar',
        target = tar,
        source = files,
        rule = tar_rule)
    bld.objects(name = name,
                target = obj,
                source = tar,
                rule = '${OBJCOPY} -I binary -B ${RTEMS_ARCH} ${OBJCOPY_FLAGS} ${SRC} ${TGT}')

def clone_tasks(bld):
    if bld.cmd == 'build':
        for obj in bld.all_task_gen[:]:
            for x in arch_bsp:
                cloned_obj = obj.clone(x)
                kind = Options.options.build_kind
                if kind.find(x) < 0:
                    cloned_obj.posted = True
            obj.posted = True

#
# From the demos. Use this to get the command to cut+paste to play.
#
def output_command_line():
    # first, display strings, people like them
    from waflib import Utils, Logs
    from waflib.Context import Context
    def exec_command(self, cmd, **kw):
        subprocess = Utils.subprocess
        kw['shell'] = isinstance(cmd, str)
        if isinstance(cmd, str):
            Logs.info('%s' % cmd)
        else:
            cmdstr = ' '.join(cmd)
            Logs.info('(%d) %s' % (len(cmdstr), cmdstr)) # here is the change
        if not isinstance(kw['cwd'], str):
            kw['cwd'] = str(kw['cwd'])
        Logs.debug('runner_env: kw=%s' % kw)
        try:
            if self.logger:
                self.logger.info(cmd)
                kw['stdout'] = kw['stderr'] = subprocess.PIPE
                p = subprocess.Popen(cmd, **kw)
                (out, err) = p.communicate()
                if out:
                    self.logger.debug('out: %s' % out.decode(sys.stdout.encoding or 'iso8859-1'))
                if err:
                    self.logger.error('err: %s' % err.decode(sys.stdout.encoding or 'iso8859-1'))
                return p.returncode
            else:
                p = subprocess.Popen(cmd, **kw)
                return p.wait()
        except OSError:
            return -1
    Context.exec_command = exec_command

    # Change the outputs for tasks too
    from waflib.Task import Task
    def display(self):
        return '' # no output on empty strings

    Task.__str__ = display

#
# From the extras. Use this to support long command lines.
#
def long_command_line():
    def exec_command(self, cmd, **kw):
        # workaround for command line length limit:
        # http://support.microsoft.com/kb/830473
        import tempfile
        tmp = None
        try:
            if not isinstance(cmd, str) and len(str(cmd)) > 8192:
                (fd, tmp) = tempfile.mkstemp(dir=self.generator.bld.bldnode.abspath())
                flat = ['"%s"' % x.replace('\\', '\\\\').replace('"', '\\"') for x in cmd[1:]]
                try:
                    os.write(fd, ' '.join(flat).encode())
                finally:
                    if tmp:
                        os.close(fd)
                # Line may be very long:
                # Logs.debug('runner:' + ' '.join(flat))
                cmd = [cmd[0], '@' + tmp]
            ret = super(self.__class__, self).exec_command(cmd, **kw)
        finally:
            if tmp:
                os.remove(tmp)
        return ret
    for k in 'c cxx cprogram cxxprogram cshlib cxxshlib cstlib cxxstlib'.split():
        cls = Task.classes.get(k)
        if cls:
            derived_class = type(k, (cls,), {})
            derived_class.exec_command = exec_command
            if hasattr(cls, 'hcode'):
                derived_class.hcode = cls.hcode

def _find_tools(conf, arch, paths, tools):
    if arch not in tools:
        arch_tools = {}
        arch_tools['CC']          = conf.find_program([arch + '-gcc'], path_list = paths)
        arch_tools['CXX']         = conf.find_program([arch + '-g++'], path_list = paths)
        arch_tools['LINK_CC']     = arch_tools['CC']
        arch_tools['LINK_CXX']    = arch_tools['CXX']
        arch_tools['AS']          = conf.find_program([arch + '-gcc'], path_list = paths)
        arch_tools['LD']          = conf.find_program([arch + '-ld'],  path_list = paths)
        arch_tools['AR']          = conf.find_program([arch + '-ar'], path_list = paths)
        arch_tools['NM']          = conf.find_program([arch + '-nm'], path_list = paths)
        arch_tools['OBJDUMP']     = conf.find_program([arch + '-objdump'], path_list = paths)
        arch_tools['OBJCOPY']     = conf.find_program([arch + '-objcopy'], path_list = paths)
        arch_tools['READELF']     = conf.find_program([arch + '-readelf'], path_list = paths)
        arch_tools['STRIP']       = conf.find_program([arch + '-strip'], path_list = paths)
        arch_tools['RANLIB']      = conf.find_program([arch + '-ranlib'], path_list = paths)
        arch_tools['RTEMS_LD']    = conf.find_program(['rtems-ld'], path_list = paths,
                                                      mandatory = False)
        arch_tools['RTEMS_TLD']   = conf.find_program(['rtems-tld'], path_list = paths,
                                                      mandatory = False)
        arch_tools['RTEMS_SYMS']  = conf.find_program(['rtems-syms'], path_list = paths,
                                                      mandatory = False)
        arch_tools['RTEMS_BIN2C'] = conf.find_program(['rtems-bin2c'], path_list = paths,
                                                      mandatory = False)
        arch_tools['TAR']         = conf.find_program(['tar'], mandatory = False)
        tools[arch] = arch_tools
    return tools

def _find_installed_archs(config, path, version):
    archs = []
    if config is None:
        for d in os.listdir(path):
            if d.endswith('-rtems' + version):
                archs += [d]
    else:
        a = subprocess.check_output([config, '--list-format', '"%(arch)s"'])
        a = a[:-1].replace('"', '')
        archs = set(a.split())
        archs = ['%s-rtems%s' % (x, version) for x in archs]
    archs.sort()
    return archs

def _check_archs(config, req, path, version):
    installed = _find_installed_archs(config, path, version)
    archs = []
    for a in req.split(','):
        arch = a + '-rtems' + version
        if arch in installed:
            archs += [arch]
    archs.sort()
    return archs

def _find_installed_arch_bsps(config, path, archs, version):
    arch_bsps = []
    if config is None:
        for f in os.listdir(_pkgconfig_path(path)):
            if f.endswith('.pc'):
                if _arch_from_arch_bsp(f[:-3]) in archs:
                    arch_bsps += [f[:-3]]
    else:
        ab = subprocess.check_output([config, '--list-format'])
        ab = ab[:-1].replace('"', '')
        ab = ab.replace('/', '-rtems%s-' % (version))
        arch_bsps = [x for x in set(ab.split())]
    arch_bsps.sort()
    return arch_bsps

def _check_arch_bsps(req, config, path, archs, version):
    archs_bsps = []
    for ab in req.split(','):
        abl = ab.split('/')
        if len(abl) != 2:
            return []
        found = False
        for arch in archs:
            a = '%s-rtems%s' % (abl[0], version)
            if a == arch:
                found = True
                break
        if not found:
            return []
        archs_bsps += ['%s-%s' % (a, abl[1])]
    if len(archs_bsps) == 0:
        return []
    installed = _find_installed_arch_bsps(config, path, archs, version)
    bsps = []
    for b in archs_bsps:
        if b in installed:
            bsps += [b]
    bsps.sort()
    return bsps

def _arch_from_arch_bsp(arch_bsp):
    return '-'.join(arch_bsp.split('-')[:2])

def _bsp_from_arch_bsp(arch_bsp):
    return '-'.join(arch_bsp.split('-')[2:])

def _pkgconfig_path(path):
    return os.path.join(path, 'lib', 'pkgconfig')

def _load_flags(conf, arch_bsp, path):
    if not os.path.exists(path):
        ctx.fatal('RTEMS path not found.')
    if os.path.exists(_pkgconfig_path(path)):
        pc = os.path.join(_pkgconfig_path(path), arch_bsp + '.pc')
        conf.to_log('Opening and load pkgconfig: ' + pc)
        pkg = pkgconfig.package(pc)
        config = None
    elif os.path.exists(os.path.join(path, 'rtems-config')):
        config = os.path.join(path, 'rtems-config')
        pkg = None
    flags = {}
    _log_header(conf)
    flags['CFLAGS'] = _load_flags_set('CFLAGS', arch_bsp, conf, config, pkg)
    flags['LDFLAGS'] = _load_flags_set('LDFLAGS', arch_bsp, conf, config, pkg)
    flags['LIB'] = _load_flags_set('LIB', arch_bsp, conf, config, pkg)
    #
    # Handle gccdeps flags.
    #
    if '-MMD' in conf.env['CFLAGS']:
        flags['CFLAGS'] += ['-MMD']
    return flags

def _load_flags_set(flags, arch_bsp, conf, config, pkg):
    conf.to_log('%s ->' % flags)
    if pkg is not None:
        flagstr = ''
        try:
            flagstr = pkg.get(flags)
        except pkgconfig.error as e:
            conf.to_log('pkconfig warning: ' + e.msg)
        conf.to_log('  ' + flagstr)
    else:
        flags_map = { 'CFLAGS': '--cflags',
                      'LDFLAGS': '--ldflags',
                      'LIB': '--libs' }
        ab = arch_bsp.split('-')
        #conf.check_cfg(path = config,
        #               package = '',
        #               uselib_store = 'rtems',
        #               args = '--bsp %s/%s %s' % (ab[0], ab[2], flags_map[flags]))
        #print conf.env
        #print '%r' % conf
        #flagstr = '-l -c'
        flagstr = subprocess.check_output([config, '--bsp', '%s/%s' % (ab[0], ab[2]), flags_map[flags]])
        #print flags, ">>>>", flagstr
        if flags == 'CFLAGS':
            flagstr += ' -DWAF_BUILD=1'
        if flags == 'LIB':
            flagstr = 'rtemscpu rtemsbsp c rtemscpu rtemsbsp'
    return flagstr.split()

def _filter_flags(label, flags, arch, rtems_path):

    flag_groups = \
        [ { 'key': 'warnings', 'path': False, 'flags': { '-W': 1 }, 'cflags': False, 'lflags': False },
          { 'key': 'includes', 'path': True,  'flags': { '-I': 1, '-isystem': 2, '-sysroot': 2 } },
          { 'key': 'libpath',  'path': True,  'flags': { '-L': 1 } },
          { 'key': 'machines', 'path': True,  'flags': { '-O': 1, '-m': 1, '-f': 1, '-G': 1, '-E': 1 } },
          { 'key': 'prepro',   'path': False, 'flags': { '-MMD': 1 } },
          { 'key': 'specs',    'path': True,  'flags': { '-q': 1, '-B': 2, '--specs': 2 } } ]

    flags = _strip_cflags(flags)

    _flags = { label: [] }
    for fg in flag_groups:
        _flags[fg['key']] = []

    iflags = iter(flags)
    for opt in iflags:
        in_label = True
        opts = []
        for fg in flag_groups:
            key = fg['key']
            for flag in fg['flags']:
                if opt.startswith(flag):
                    opt_count = fg['flags'][flag]
                    if opt_count > 1:
                        if opt != flag:
                            opt_count -= 1
                            if fg['path'] and arch in opt:
                                opt = '%s%s/%s' % (flag, rtems_path,
                                                   opt[opt.find(arch):])
                    opts += [opt]
                    for c in range(1, opt_count):
                        opt = next(iflags)
                        if fg['path'] and arch in opt:
                            opt = '%s%s/%s' % (f, rtems_path,
                                               opt[opt.find(arch):])
                        opts += [opt]
                    _flags[key] += opts
                    if label in fg and not fg[label]:
                        in_label = False
                    break
        if in_label:
            _flags[label] += opts
    return _flags

def _strip_cflags(cflags):
    _cflags = []
    for o in cflags:
        if o.startswith('-O'):
            pass
        elif o.startswith('-g'):
            pass
        else:
            _cflags += [o]
    return _cflags

def _log_header(conf):
    conf.to_log('-----------------------------------------')

def _get_dir_hash(bld):
    from waflib import ConfigSet, Options
    import hashlib

    env = ConfigSet.ConfigSet()
    env.load(Options.lockfile)
    prefix = env.options['prefix']
    shahash = hashlib.sha1()

    for root, dirs, files in os.walk(prefix):
        for names in files:
            filepath = os.path.join(root, names)
            try:
                f1 = open(filepath, 'rb')
            except:
                f1.close()
                continue

            while 1:
                buf = f1.read(4096)
                if not buf:
                    break
                shahash.update(hashlib.sha1(buf).hexdigest())
            f1.close()
    return shahash.hexdigest()

def test_uninstall(bld):
    from os import sys

    print('Test: uninstall')
    initial_hash = _get_dir_hash(bld)
    print('Preinstall hash: %s' % (initial_hash))
    try:
        subprocess.call(['waf', 'install'])
        subprocess.call(['waf', 'uninstall'])
    except:
        subprocess.call(['./waf', 'install'])
        subprocess.call(['./waf', 'uninstall'])
    final_hash = _get_dir_hash(bld)
    print('Post install hash: %s' % (final_hash))

    if (initial_hash == final_hash):
        print("Test successful")
    else:
        print("Test failed")

from waflib import Task
from waflib import TaskGen
from waflib import Utils
from waflib import Node
from waflib.Tools.ccroot import link_task, USELIB_VARS

USELIB_VARS['rap'] = set(['RTEMS_LINKFLAGS'])
USELIB_VARS['rtrace'] = set(['RTRACE_FLAGS', 'RTRACE_CFG', 'RTRACE_WRAPPER', 'RTRACE_LINKCMDS'])

class rap(link_task):
    "Link object files into a RTEMS application"
    run_str = '${RTEMS_LD} ${RTEMS_LINKFLAGS} --cc ${CC} ${SRC} -o ${TGT[0].abspath()} ${STLIB_MARKER} ${STLIBPATH_ST:STLIBPATH} ${STLIB_ST:STLIB} ${LIBPATH_ST:LIBPATH} ${LIB_ST:LIB}'
    ext_out = ['.rap']
    vars    = ['RTEMS_LINKFLAGS', 'LINKDEPS']
    inst_to = '${BINDIR}'

class rtrace(link_task):
    "Link object files into a RTEMS trace application"
    run_str = '${RTEMS_TLD} ${RTACE_FLAGS} ${RTRACE_WRAPPER_ST:RTRACE_WRAPPER} -C ${RTRACE_CFG} -r ${RTEMS_PATH} -B ${ARCH_BSP} -c ${CC} -l ${CC} -- ${SRC} ${LINKFLAGS} ${RTRACE_LINKFLAGS} -o ${TGT[0].abspath()} ${STLIB_MARKER} ${STLIBPATH_ST:STLIBPATH} ${STLIB_ST:STLIB} ${LIBPATH_ST:LIBPATH} ${LIB_ST:LIB}'
    ext_out = ['.texe']
    vars    = ['RTRACE_FLAGS', 'RTRACE_CFG', 'RTRACE_WRAPER', 'RTRACE_LINKFLAGS', 'LINKDEPS']
    inst_to = '${BINDIR}'
    color = 'PINK'
