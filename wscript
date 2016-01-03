#! /usr/bin/env python
# encoding: utf-8
TOP = '.'
  
def options(opt):
    opt.recurse(['externals/ecpp'])

def configure(conf):
    conf.recurse(['externals/ecpp','firmware'])
  
def build(ctx):
    ctx.recurse(['externals/ecpp','firmware'])