# import the python files in subdirectories 
# for example
#   filters 
#       |--lowpass
#       |--ex_kalman
#       |--luenberger 
# 

from os.path import dirname, basename, isfile, join
import glob

# idirs = ['D:\\gh_repo\\filters'
#             , 'D:\\gh_repo\\filters\\lowpass'
#             , 'D:\\gh_repo\\filters\\ex_kalman'
#             , 'D:\\gh_repo\\filters\\luenberger']

def importsubdir(idirs):
    modules = ''.join(',' + ','.join(glob.glob(join(d, "*.py"))) for d in idirs)
    __all__ = [basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')] # exclude folders and the init file 

# for f in modules.split(','):
#     if isfile(f) and not f.endswith('__init__.py'):
#         print(f)


# for f in modules.split(','):
#     print(f)









