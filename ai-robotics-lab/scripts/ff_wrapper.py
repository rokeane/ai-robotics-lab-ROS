#!/usr/bin/env python

from ai_robotics_lab.srv import FFSolve, FFSolveResponse
import rospy
from pathlib2 import Path
import tempfile
import subprocess
import json

def pddl_file(pddl, folder=None):
    if pddl.endswith('.pddl'):
        p = Path(pddl)
        return str(p.parent), p.name
    else:
        f = tempfile.NamedTemporaryFile(delete=False,prefix='lab',suffix='.pddl',dir=folder)
        p = Path(f.name)
        f.write(pddl)
        f.close()
        return str(p.parent), p.name

def ff(folder, domain, problem, ff_exec='/home/lesire/Sources/lecturer/FF-v2.3/ff'):
    ffp = subprocess.Popen([ff_exec, '-p', folder+'/', '-o', domain, '-f', problem], stdout=subprocess.PIPE)
    result, errors = ffp.communicate()
    return result

def parse_ff_result(result):
    def parse_line(l):
        i = l.find(':')
        return l[i-1], l[i+2:].split(" ")
    lines = result.split('\n')
    b = False
    for l in lines:
        if 'step' in l:
            b = True
        elif 'time spent' in l:
            b = False
        if b and ':' in l:
            yield parse_line(l)

class FFWrapper:
    def __init__(self):
        s = rospy.Service('solve', FFSolve, self.ff_request)
        self.ff_path = rospy.get_param("~ff_path", '/home/etdisc/sources/FF-v2.3/ff')
        rospy.loginfo("Calling FF %s" % self.ff_path)
        subprocess.call([self.ff_path, '--help'])

    def ff_request(self, req):
        folder, domain = pddl_file(req.domain)
        _, problem = pddl_file(req.problem, folder=folder)
        rospy.loginfo("Launch FF on %s, %s" % (domain, problem))
        solution = ff(folder, domain, problem, ff_exec=self.ff_path)
        resp = json.dumps(list(parse_ff_result(solution)))
        return FFSolveResponse(resp)

def main():
    rospy.init_node("FF")
    ff = FFWrapper()
    rospy.spin()

if __name__ == "__main__":
    main()
