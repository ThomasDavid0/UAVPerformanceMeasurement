"""https://stackoverflow.com/questions/58768597/how-can-i-bundle-lua-scripts-to-single-file"""
import argparse
from pathlib import Path
import re


parser = argparse.ArgumentParser(
    prog = 'LUABundle',
    description='Bundle LUA Scripts for ArduPilot'
)

parser.add_argument('-s', '--source', default='src/main_parabolic.lua')
parser.add_argument('-t', '--target', default='sitl/scripts/bundle.lua')
parser.add_argument('-w', '--workdir', default='src')

args = parser.parse_args()

source = Path(args.source)
target = Path(args.target)
workdir = Path(args.workdir)

all_lua_files = sorted(list(workdir.rglob('*.lua')))


required_files = {}


def parse_require_statement(line):
    '''
        local *** = require("sdxvsvd")
        local *** = require "sdxvsvd"    
    '''
    cols = line.split('require')
    assert len(cols) <= 2
    
    rhs = cols[-1]#.replace('require','')
    
    file = rhs.strip("('')"" \n")
    if not '.lua' in file: 
        file = file + '.lua'
        
    fname = file[:-4]
    
    if not file in required_files:
        required_files[fname] = parse_file(workdir / file)
    

def parse_file(file):
    with open(file, 'r') as f:
        source_lines = f.readlines()
        
    require_ids = [i for i, l in enumerate(source_lines) if re.search(r"\b" + re.escape('require') + r"\b", l)]

    for rlid in require_ids:
        parse_require_statement(source_lines[rlid])
    
    return source_lines



with open('bundle_base.lua', 'r') as f:
    output = f.readlines()
    
original_lines = parse_file(source)

for fname, flines in required_files.items():
    output.append('')
    output.append(f"files['{fname}'] = function(...)\n")
    output = output + [f'    {l}' for l in flines]
    output[-1] = output[-1] + '\n'
    output.append('end\n')

output = output + original_lines

with open(args.target, 'w') as f:
    f.writelines(output)


