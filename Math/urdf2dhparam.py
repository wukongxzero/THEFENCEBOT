import os
import xml.etree.ElementTree as ET
from sympy import Matrix, symbols, pi,nsimplify

def parse_urdf_to_dh_matrix(urdf_path:str,angles=[]):
    # 1. Find the first .urdf file in the current directory

    #used googl;e ai, am changing this to just take a file name
    if urdf_path == None or urdf_path=="":
        urdf_files = [f for f in os.listdir('.') if f.endswith('.urdf')]
        urdf_path = urdf_files[0]
  
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    dh_params = []
    dh_param_human = []
    # 2. Iterate through joints to find origin offsets
    # URDF origins are defined relative to the parent link
    for joint in root.findall('joint'):
        name = joint.get('name')
        origin = joint.find('origin')
        
        # Default values if <origin> is missing
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]

        if origin is not None:
            xyz_str = origin.get('xyz', '0 0 0')
            rpy_str = origin.get('rpy', '0 0 0')
            xyz = [float(x) for x in xyz_str.split()]
            rpy = [float(r) for r in rpy_str.split()]

        # Mapping to your requested DH-like state:
        # theta: Rotation around Z (rpy[2])
        # d: Distance along Z (xyz[2])
        # a: Distance along X (xyz[0])
        # alpha: Rotation around X (rpy[0])
        
        theta = rpy[2]+ angles[len(dh_params)]
        d = xyz[2]
        a = xyz[0]
        alpha = rpy[0]

        dh_params.append([theta, d, a, alpha])

        theta_sym = nsimplify(theta / 3.1415926535897932, tolerance=1e-1) * pi
        alpha_sym = nsimplify(alpha / 3.1415926535897932, tolerance=1e-1) * pi
    
        
        dh_param_human.append([theta_sym, d, a, alpha_sym])


    if not dh_params:
        print("No joints found in URDF.")
        return None

    # 3. Construct the SymPy Matrix
    # Each column is (theta, d, a, alpha)^T
    # We transpose the list of lists to make parameters rows and joints columns
    param_matrix = Matrix(dh_params)
    dh_param_human = Matrix(dh_param_human)
    return param_matrix,dh_param_human, urdf_path

# Execute and display
#result = parse_urdf_to_dh_matrix()
#if result:
#    mat, filename = result
#    print(f"Extracted Matrix from {filename}:")
#   # display(mat) # Works in Jupyter; use print(mat) in standard scripts
#    print(mat)