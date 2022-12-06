#
# RTEMS Project (https://www.rtems.org/)
#
# Copyright (c) 2016 Chris Johns <chrisj@rtems.org>. All rights reserved.
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

from __future__ import print_function

import os.path

try:
    import rtems_waf.rtems as rtems
except:
    print("error: no rtems_waf module")
    import sys
    sys.exit(1)

def init(ctx):
    pass

def options(opt):
    opt.add_option('--net-config',
                   default = 'config.inc',
                   dest = 'net_config',
                   help = 'Network test configuration.')
    opt.add_option('--rtems-libbsd',
                   action = 'store',
                   default = None,
                   dest = 'rtems_libbsd',
                   help = 'Path to install RTEMS LibBSD (defauls to prefix).')

def bsp_configure(conf, arch_bsp, mandatory = True):
    configure = mandatory

    if not mandatory and conf.options.rtems_libbsd is not None:
        configure = True

    if configure:
        conf.msg('RTEMS LibBSD',
                 rtems.arch(arch_bsp) + '/' + rtems.bsp(arch_bsp),
                 'YELLOW')

        conf.check(header_name = 'dlfcn.h', features = 'c')
        if not rtems.check_posix(conf):
            conf.fatal('RTEMS kernel POSIX support is disabled; ' +
                       'configure RTEMS with --enable-posix')
        if rtems.check_networking(conf):
            conf.fatal('RTEMS kernel contains the old network support; ' +
                       'configure RTEMS with --disable-networking')
        rtems_libbsd_path = conf.options.rtems_libbsd
        if rtems_libbsd_path is None:
            if conf.options.rtems is None:
                rtems_libbsd_path = conf.options.rtems
            else:
                rtems_libbsd_path = conf.env.PREFIX

        if not os.path.exists(rtems_libbsd_path):
                conf.fatal('RTEMS LibBSD path not found: %s' % (rtems_libbsd_path))

        rtems_libbsd_inc_path = os.path.join(rtems_libbsd_path,
                                             rtems.arch_bsp_include_path(conf.env.RTEMS_VERSION,
                                                                     conf.env.RTEMS_ARCH_BSP))
        rtems_libbsd_lib_path = os.path.join(rtems_libbsd_path,
                                             rtems.arch_bsp_lib_path(conf.env.RTEMS_VERSION,
                                                                     conf.env.RTEMS_ARCH_BSP))

        conf.env.IFLAGS += [rtems_libbsd_inc_path]
        conf.check(header_name = 'machine/rtems-bsd-sysinit.h',
                   features = 'c',
                   includes = conf.env.IFLAGS)

        conf.env.RTEMS_LIBBSD = 'Yes'
        conf.env.INCLUDES = conf.env.IFLAGS
        conf.env.LIBPATH += [rtems_libbsd_lib_path]
        conf.env.LIB += ['bsd', 'z', 'm']

        configure_net_config(conf, arch_bsp)

def configure_net_config(conf, arch_bsp):
    if check_libbsd(conf) and conf.options.net_config is not None:
        net_config = conf.options.net_config

        if not os.path.exists(net_config):
            conf.fatal('network configuraiton \'%s\' not found' % (net_config))

        try:
            net_cfg_lines = open(net_config).readlines()
        except:
            conf.fatal('network configuraiton \'%s\' read failed' % (net_config))

        tags = [ 'NET_CFG_SELF_IP',
                 'NET_CFG_NETMASK',
                 'NET_CFG_PEER_IP',
                 'NET_CFG_GATEWAY_IP' ]

        lc = 0
        sed = 'sed '
        defines = []
        for l in net_cfg_lines:
            lc += 1
            if l.strip().startswith('NET_CFG_'):
                ls = l.split('=')
                if len(ls) != 2:
                    conf.fatal('network configuraiton \'%s\' ' + \
                              'parse error: %d: %s' % (net_config, lc, l))
                lhs = ls[0].strip()
                rhs = ls[1].strip()
                for t in tags:
                    if lhs.startswith(t):
                        conf.env[lhs] = rhs
                        sed += "-e 's/@%s@/%s/'" % (lhs, rhs)
                        defines += ['%s="%s"' % (lhs, rhs)]

        conf.env.NET_CONFIG = net_config
        conf.env.NET_CONFIG_SED = sed
        conf.env.NET_CONFIG_DEFINES = ','.join(defines)

        conf.msg('Net Config', 'found', 'YELLOW')

def check_libbsd(ctx):
    return rtems.check(ctx, 'RTEMS_LIBBSD')

def check_net_config(ctx):
    return rtems.check(ctx, 'NET_CONFIG', setting = True)

def net_config_header(ctx, target):
    ctx(target = target,
        source = "rtems_waf/network-config.h.in",
        rule = sed + " < ${SRC} > ${TGT}",
        update_outputs = True)
