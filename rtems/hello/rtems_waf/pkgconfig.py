# Copyright 2013 Chris Johns (chrisj@rtems.org)
# 
# This file's license is 2-clause BSD as in this distribution's LICENSE.2 file.
#

#
# Pkg-config in python. Pkg-config as a tool is a good idea how-ever the
# implementation is really Linux (or Unix) based and requires a couple of
# packages that it should not. If it was implemented with all parts included it
# would be portable and I suspect useful to others on platforms other than
# Linux or Unix equivs that contain the required packages.
#
import re

class error(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg

class package:
    def __init__(self, file = None):
        self.defines = {}
        self.fields = {}
        if file:
            self.load(file)

    def load(self, file):
        f = open(file)
        tm = False
        for l in f.readlines():
            l = l[:-1]
            hash = l.find('#')
            if hash >= 0:
                l = l[:hash]
            if len(l):
                d = 0
                define = False
                eq = l.find('=')
                dd = l.find(':')
                if eq > 0 and dd > 0:
                    if eq < dd:
                        define = True
                        d = eq
                    else:
                        define = False
                        d = dd
                elif eq >= 0:
                    define = True
                    d = eq
                elif dd >= 0:
                    define = False
                    d = dd
                if d > 0:
                    lhs = l[:d].lower()
                    rhs = l[d + 1:]

                    if tm:
                        print('define: ' + str(define) + ', lhs: ' + lhs + ', ' + rhs)

                    if define:
                        self.defines[lhs] = rhs
                    else:
                        self.fields[lhs] = rhs

    def get(self, label):
        if label.lower()  not in self.fields:
            raise error('Label not found: ' + label)
        mre = re.compile('\$\{[^\}]+\}')
        s = self.fields[label.lower()]
        expanded = True
        tm = False
        while expanded:
            expanded = False
            if tm:
                print('pc:get: "' + s + '"')
            ms = mre.findall(s)
            for m in ms:
                mn = m[2:-1]
                if mn.lower() in self.defines:
                    s = s.replace(m, self.defines[mn.lower()])
                    expanded = True
        return s
