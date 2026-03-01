import sympy as sp
from sympy.physics.mechanics import ReferenceFrame, Vector, inertia
#You said
#give me a python structure for holding sympy vectors where we hold a refrence point form 
# center of mass, and a moment of inertia tensor


class RigidBodyRobotLink:
    def __init__(self, name, frame=None):
        """
        Initializes a structure for tracking a body's inertia and relative position.
        
        :param name: String name for the body (e.g., 'Link1')
        :param frame: A sympy.physics.mechanics.ReferenceFrame. 
                      If None, a new one is created.
        """
        self.name = name
        self.frame = frame if frame else ReferenceFrame(f'N_{name}')
        
        # Position vector from Center of Mass (CoM) to the Reference Point
        # Initialized to the zero vector
        self.r_com = Vector(0)
        
        # Moment of Inertia Tensor (3x3 Matrix or Inertia Object)
        self.inertia_tensor = sp.Matrix.zeros(3, 3)

    def set_inertia(self, ixx, iyy, izz, ixy=0, iyz=0, izx=0):
        """Sets the inertia tensor components relative to the frame."""
        self.inertia_tensor = inertia(self.frame, ixx, iyy, izz, ixy, iyz, izx)

    def set_com_offset(self, x, y, z):
        """Sets the vector from the Center of Mass to your Reference Point."""
        self.r_com = x * self.frame.x + y * self.frame.y + z * self.frame.z

    def __repr__(self):
        return (f"RigidBodyData({self.name})\n"
                f"  CoM Offset: {self.r_com}\n"
                f"  Inertia: {self.inertia_tensor}")