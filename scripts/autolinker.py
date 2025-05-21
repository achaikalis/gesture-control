import os

Import("env")

def add_all_lib_dirs(env, lib_base):
    for root, dirs, files in os.walk(lib_base):
        if any(f.endswith(('.a', '.lib')) for f in files):
            env.Append(LIBPATH=[root])
            for f in files:
                if f.endswith(('.a', '.lib')):
                    env.Append(LINKFLAGS=[f"-l:{f}"])

def before_build(source, target, env):
    add_all_lib_dirs(env, "lib/bsxlite_interface/lib")

env.AddPreAction("buildprog", before_build)