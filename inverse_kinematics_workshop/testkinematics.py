from math import pi
import numpy as np; np.set_printoptions(precision=4, suppress = True)

from kinematicchain import Link, FixedLink, KinematicChain
from inversekinematics import InverseKinematicsSolver, pseudo_inverse

links = [
	Link(0, pi/2, 3.0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 3.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 3.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 3.0, 0),
#	Link(0, -pi/2, 0),
#	FixedLink(0, pi/2, 3.0, 0),
#	Link(0, -pi/2, 0),
#	FixedLink(0, pi/2, 2.0, 0),
#	Link(0, -pi/2, 0),
#	FixedLink(0, pi/2, 2.0, 0),
]

links = [Link(0, pi/2, 0.1)]
for i in range(170):
	links.extend([
			Link(0, -pi/2, 0),
			FixedLink(0, pi/2, 0.1, 0)
			])

chain = KinematicChain(links)

print(chain.links)

print(chain.compute_htms())

ik = InverseKinematicsSolver(chain, dt=1e-3, lr=1e-4, dt_max=0.05)

#ik.make_video([np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(0, 10)], render_every=1)
ik.make_video(render_every=1, resample_every=500, steps=1000)
