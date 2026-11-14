"""Tiny assembler for VM1 (see main/include/appvm.hpp). Expressions are written in postfix:
   a.e('nx', 10, '*')  pushes variable nx, 10, multiplies.  Strings that are not operators are variable names;
   '@xs' loads xs[<index on stack>]."""
import struct
OPS = dict(HALT=0, DUP=3, DROP=4, SWAP=5, OVER=6, ADD=10, SUB=11, MUL=12, DIV=13, MOD=14, AND=15, OR=16, XOR=17, NEG=18, NOT=19,
           LT=20, GT=21, EQ=22, NE=23, LE=24, GE=25, LOAD=40, STORE=41, RND=50, KEY=51, FRAME=52, CLS=53, RECT=54, NUM=55, SYS=60, STR=61)
SYM = {'+': 'ADD', '-': 'SUB', '*': 'MUL', '/': 'DIV', '%': 'MOD', '&': 'AND', '|': 'OR', '^': 'XOR', '<': 'LT', '>': 'GT',
       '==': 'EQ', '!=': 'NE', '<=': 'LE', '>=': 'GE', 'neg': 'NEG', 'not': 'NOT', 'dup': 'DUP', 'drop': 'DROP', 'swap': 'SWAP',
       'over': 'OVER', 'key': 'KEY', 'rnd': 'RND'}
class Asm:
    def __init__(s): s.b = bytearray(); s.labels = {}; s.fix = []; s.vars = {}; s.top = 0
    def var(s, name, size=1): s.vars[name] = (s.top, size); s.top += size; assert s.top <= 1280; return s.vars[name][0]
    def push(s, n):
        assert -32768 <= n <= 32767
        if -128 <= n <= 127: s.b += bytes([1, n & 255])
        else: s.b += bytes([2]) + struct.pack('<h', n)
    def op(s, name): s.b.append(OPS[name])
    def label(s, name): assert name not in s.labels; s.labels[name] = len(s.b)
    def _jump(s, code, name): s.b.append(code); s.fix.append((len(s.b), name)); s.b += b'\0\0'
    def jmp(s, n): s._jump(30, n)
    def jz(s, n): s._jump(31, n)
    def jnz(s, n): s._jump(32, n)
    def e(s, *toks):
        for t in toks:
            if isinstance(t, int): s.push(t)
            elif t in SYM: s.op(SYM[t])
            elif t.startswith('@'): s.push(s.vars[t[1:]][0]); s.op('ADD'); s.op('LOAD')
            else: s.push(s.vars[t][0]); s.op('LOAD')
    def st(s, name): s.push(s.vars[name][0]); s.op('STORE')            # value on stack -> var
    def set(s, name, *expr): s.e(*expr); s.st(name)
    def sta(s, name, val_expr, idx_expr):                              # name[idx] = val
        s.e(*val_expr); s.e(*idx_expr); s.push(s.vars[name][0]); s.op('ADD'); s.op('STORE')
    def text(s, x, y, color, msg):
        s.push(x); s.push(y); s.push(color); b = msg.encode('ascii'); assert len(b) <= 40; s.b += bytes([56, len(b)]) + b
    def rect(s, x, y, w, h, color):                                    # each arg: int or list of tokens
        for a in (x, y, w, h, color): s.e(*(a if isinstance(a, (list, tuple)) else [a]))
        s.op('RECT')
    def str(s, name, msg):                                             # store msg (one char per word, 0 terminated) at variable `name`
        b = msg.encode('ascii'); assert len(b) <= 80 and s.vars[name][1] >= len(b) + 1
        s.push(s.vars[name][0]); s.b += bytes([61, len(b)]) + b
    def sys(s, fn, a=0, b=0, c=0):                                     # device call; leaves its result on the stack
        for arg in (a, b, c): s.e(*(arg if isinstance(arg, (list, tuple)) else [arg]))
        s.push(fn); s.op('SYS')
    def finish(s):
        for pos, name in s.fix: s.b[pos:pos + 2] = struct.pack('<H', s.labels[name])
        assert len(s.b) <= 4096, len(s.b)
        return bytes(s.b)
def make_app(name, category, version, desc, code):
    return ("MARAUDER_APP=1\nAPI=1\nRUNTIME=VM1\nNAME=%s\nCATEGORY=%s\nVERSION=%s\nDESCRIPTION=%s\nCODE=%s\n"
            % (name, category, version, desc, code.hex().upper()))
