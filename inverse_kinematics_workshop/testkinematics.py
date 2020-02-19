from math import pi
import numpy as np; np.set_printoptions(precision=4, suppress = True)

from kinematicchain import Link, FixedLink, KinematicChain
from inversekinematics import InverseKinematicsSolver, pseudo_inverse

links = [
	Link(0, pi/2, 0.0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 2.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 2.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 2.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 2.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 2.0, 0),
	Link(0, -pi/2, 0),
	FixedLink(0, pi/2, 2.0, 0),
]

chain = KinematicChain(links)

print(chain.links)

print(chain.compute_htms())

ik = InverseKinematicsSolver(chain, dt=1e-2, lr=1e-4, dt_max=0.1, max_steps=1000)

ik.set_target([np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(0, 10)])

ik.make_video([np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(0, 10)], render_every=1)
