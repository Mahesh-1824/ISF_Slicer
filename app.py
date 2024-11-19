from flask import Flask, request, render_template, redirect, url_for, send_from_directory, send_file, jsonify,session, flash
import os
import numpy as np
import plotly.graph_objects as go
import math
import zipfile
import io
from flask_sqlalchemy import SQLAlchemy
from werkzeug.security import generate_password_hash, check_password_hash
from datetime import datetime
from werkzeug.utils import secure_filename
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopoDS import TopoDS_Vertex, TopoDS_Edge
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCC.Core.BRep import BRep_Tool
from OCC.Core.GCPnts import GCPnts_UniformAbscissa
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.GeomLib import GeomLib_Tool
from OCC.Core.GeomAPI import GeomAPI_IntCS, GeomAPI_ProjectPointOnSurf
from OCC.Core.TopoDS import topods, TopoDS_Shape, TopoDS_Face
from OCC.Core.Geom import Geom_Curve, Geom_Plane
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.GCPnts import GCPnts_UniformAbscissa
from OCC.Core.gp import gp_Pnt, gp_Pln, gp_Ax3, gp_Dir, gp_Vec, gp_XYZ
from OCC.Core.TopExp import topexp
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Edge, shapeanalysis
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Vertex, TopoDS_Edge
from OCC.Core.TopAbs import TopAbs_VERTEX
# from OCC.Core.Standard import Standard_Boolean
# from gen_htp import FindStart
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_EDGE
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh 
from OCC.Core.BRepBndLib import brepbndlib_Add
from datetime import datetime
BASE_PATH = "./"

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'postgresql+psycopg2://postgres:postgres@localhost:5432/toolpath_db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False  # Optional to silence warnings
db = SQLAlchemy(app)
class User(db.Model):
    __tablename__ = 'isf_users'
    id = db.Column(db.Integer, primary_key=True)
    first_name = db.Column(db.String(50), nullable=False)
    last_name = db.Column(db.String(50), nullable=False)
    email = db.Column(db.String(100), unique=True, nullable=False)
    password = db.Column(db.String(255), nullable=False)
    created_at = db.Column(db.DateTime, default=db.func.current_timestamp())
    def __repr__(self):
        return f"<User {self.first_name} {self.last_name}>"
class Feedback(db.Model):
    __tablename__ = 'feedback'
    id = db.Column(db.Integer, primary_key=True)
    user_id = db.Column(db.Integer, db.ForeignKey('isf_users.id'), nullable=False)
    user_name = db.Column(db.String(100), nullable=False)  # New field for user name
    comment = db.Column(db.Text, nullable=False)
    timestamp = db.Column(db.DateTime, default=db.func.current_timestamp())
    
    user = db.relationship('User', backref=db.backref('feedback', lazy=True))

    def __repr__(self):
        return f"<Feedback from User {self.user_id}: {self.comment}>"
with app.app_context():
    db.create_all()  # Creates tables based on the models defined
#initializing all the directories
UPLOAD_FOLDER='uploads'
ALLOWED_EXTENSIONS = {'step', 'stp'}
app.config['UPLOAD_FOLDER']=UPLOAD_FOLDER
app.secret_key = os.urandom(24) 
#specifying the desired file name
def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS
date_time=str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
#trigering index.html file to open the landing page
@app.route('/')
def index():
    return render_template('landing.html')

#initializing user email for creating folder
email=str()
name=str()

@app.route('/login')
def login():
    return render_template('login.html')

sspiral=np.empty((2,3))
scontour=np.empty((2,3))


@app.route('/upload', methods=['POST'])
def upload_file():
    global email
    
    # Check if file is part of the request
    if 'file' not in request.files:
        return redirect(request.url)
    
    file = request.files['file']
    if file.filename == '':
        return redirect(request.url)
    
    # Proceed if the file is allowed (STEP or STP format)
    if file and allowed_file(file.filename):
        filename = email[:email.index('@')]
        step_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(step_path)
        
        # Generate a unique timestamp for each upload
        date_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        
        # Correct directory paths using os.path.join
        global contour_folder, spiral_folder
        contour_folder = BASE_PATH + 'users/' +email+  '/Contour_' + date_time
        spiral_folder =  app.config['USER_FOLDER'] +  '/Spiral' + date_time
        print(contour_folder)
        print(spiral_folder)
        # Create the folders if they don't exist
        os.makedirs(contour_folder, exist_ok=True)
        os.makedirs(spiral_folder, exist_ok=True)
        
        # User-provided inputs
        TD1 = request.form['tool_dia']
        Feed = request.form['feedrate']
        cnc = request.form['cnc']
        
        # Load STEP file
        shape = load_step(step_path)
        if shape is None:
            return "Error: Failed to load or read STEP file", 400
        
        # Convert STEP to STL
        stl_filename = convert_step_to_stl(step_path, shape)
        
        # Define paths for toolpath files within the created folders
        pnt_contour_path =contour_folder + "/pntContour.txt"
        n_contour_path = contour_folder + "/nContour.txt"
        pnt_spiral_path = spiral_folder+ "/pntSpiral.txt"
        n_spiral_path = spiral_folder+ "/nSpiral.txt"
        print(pnt_contour_path)
        # Process STEP file to generate required files
        process_step_file(step_path, contour_folder, spiral_folder)

        # Generate toolpaths for contour and spiral
        gen_toolpath(pnt_contour_path, n_contour_path, TD1, Feed, cnc, 'contourSPIF_', contour_folder)
        gen_toolpath(pnt_spiral_path, n_spiral_path, TD1, Feed, cnc, 'spiralSPIF_', spiral_folder)
        
        global scontour, sspiral
        contour_html_path =BASE_PATH + "static/pnt.html"
        spiral_html_path = BASE_PATH + "static/spnt.html"
        
        # Generate plots for the contour and spiral trajectories
        scontour = plot(pnt_contour_path, contour_html_path, 'Contour Trajectory')
        sspiral = plot(pnt_spiral_path, spiral_html_path, 'Spiral Trajectory')
        
        # Redirect to the STL file view
        return redirect(url_for('view_file', filename=stl_filename))
    
    # Handle invalid file type
    else:
        message2 = 'Welcome ' + name
        return render_template('index.html', message1='Please choose a .STEP or .STP file', message=message2)




@app.route('/create-folder/<folder_name>')
def create_subfolder(folder_name):
    base_dir = app.config['USER_FOLDER']
    folder_path = os.path.join(base_dir, folder_name)
    os.makedirs(folder_path, exist_ok=True)

    return folder_name


@app.route('/createaccount')
def signup_page():
    return render_template('signup.html')

USER_FOLDER=str()
@app.route('/signup', methods=['POST'])
def signup():
    global USER_FOLDER, name, email
    name = request.form['Name']
    lname = request.form['lname']
    email = request.form['email']
    password = request.form['password1']
    cnfpassword = request.form['password2']
    
    if password == cnfpassword and len(password) > 8:
        # Check if email already exists in the database
        existing_user = User.query.filter_by(email=email).first()
        if existing_user:
            return render_template('signup.html', message="Email already registered")

        # Hash the password and create a new user using pbkdf2:sha256
        hashed_password = generate_password_hash(password, method='pbkdf2:sha256')
        new_user = User(first_name=name, last_name=lname, email=email, password=hashed_password)
        db.session.add(new_user)
        db.session.commit()

        # Create the user's folder
        BASE_DIR = BASE_PATH + "users"
        USER_FOLDER = os.path.join(BASE_DIR, email)
        app.config['USER_FOLDER'] = USER_FOLDER
        if not os.path.exists(USER_FOLDER):
            os.makedirs(USER_FOLDER)
        
        # Redirect to login page after signup
        return render_template('login.html', message="Signup successful! Please log in.")
    else:
        return render_template('signup.html', message="Check password requirements")


@app.route('/signin', methods=['POST', 'GET'])
def signin():
    if request.method == 'POST':
        global name, email, USER_FOLDER
        email = request.form['email']
        password = request.form['password']
        
        user = User.query.filter_by(email=email).first()
        
        if user and check_password_hash(user.password, password):
            name = user.first_name
            session['user_email'] = email  # Store email in session
            session['user_name'] = name  # Store user name in session
            
            # Set up user folder
            BASE_DIR = BASE_PATH + "users"
            USER_FOLDER = os.path.join(BASE_DIR, email)
            app.config['USER_FOLDER'] = USER_FOLDER
            if not os.path.exists(USER_FOLDER):
                os.makedirs(USER_FOLDER)
            
            # Redirect to the /home route after successful signin
            return redirect(url_for('home'))
        else:
            message = 'Invalid email or password'
            return render_template('login.html', message=message)
    return render_template('login.html')


@app.route('/home', methods=['GET'])
def home():
    # Check if the session is valid
    if 'user_email' in session and 'user_name' in session:
        message = f'Welcome {session["user_name"]}'
        return render_template('index.html', message=message)
    else:
        return redirect(url_for('signin'))  # Redirect to login if session is invalid

@app.route('/signout', methods=['GET', 'POST'])
def signout():
    # Check if the user confirmed the signout
    print(session['user_email'])
    print(session['user_name'])
    if request.method == 'POST':
        confirm = request.form.get('confirm')
        if confirm == 'yes':
            session.clear()  # Clear the user session
            return redirect(url_for('login'))  # Redirect to the landing page
        else:
            return redirect(url_for('home'))
    return render_template('signout.html')  # Render confirmation page
        
#loading the .step file uploded by user
def load_step(file_path):
    try:
        reader = STEPControl_Reader()
        status = reader.ReadFile(file_path)
        if status == 1:
            reader.TransferRoots()
            shape = reader.Shape()
            return shape
        else:
            return None
    except Exception as e:
        print(f"Error reading .step file: {e}")
        return None

#converting to stl file(required for display only)
def convert_step_to_stl(step_path, shape):
    mesh = BRepMesh_IncrementalMesh(shape, 0.1)
    mesh.Perform()
    stl_writer = StlAPI_Writer()
    stl_filename = os.path.splitext(step_path)[0] + '.stl'
    stl_writer.Write(shape, stl_filename)
    return os.path.basename(stl_filename)


def FindStart(edges, ordered_edges, start_point):
    distance = 100000
    startEdgeNo = 0
    nEdges = len(edges)
    for i in range(0,nEdges):
        iEdge = edges[i]
        v1, v2 = get_edge_vertices(iEdge)
        pnt_v1 = BRep_Tool.Pnt(v1)
        dist1 = pnt_v1.Distance(start_point)
        if(dist1 < distance):
            startEdgeNo = i
            distance = dist1
    aEdge = edges[startEdgeNo]
    edges[startEdgeNo] = edges[0]
    edges[0] = aEdge

    for i in range(0,nEdges):
        ordered_edges.append(edges[i])
    return ordered_edges

def stPnt(st_pnt,edges):
    ordered_edges=[]
    
    return FindStart(edges,ordered_edges,st_pnt)
def get_edge_vertices(aEdge):
    # Initialize an explorer to find vertices
    explorer = TopExp_Explorer(aEdge, TopAbs_VERTEX)

    # Get the first vertex
    first_vertex = None
    last_vertex = None
    if explorer.More():
        first_vertex = TopoDS_Vertex(explorer.Current())
        explorer.Next()
    
    # Get the last vertex
    while explorer.More():
        last_vertex = TopoDS_Vertex(explorer.Current())
        explorer.Next()
    
    return first_vertex, last_vertex

def vertices_are_equal(vertex1, vertex2, tolerance):
    # Extract 3D coordinates from the vertices
    p1 = BRep_Tool.Pnt(vertex1)
    p2 = BRep_Tool.Pnt(vertex2)

    # Compare the coordinates of the two points with tolerance
    return (abs(p1.X() - p2.X()) < tolerance and
            abs(p1.Y() - p2.Y()) < tolerance and
            abs(p1.Z() - p2.Z()) < tolerance)

def EdgesOrdering(slice_edges, ordered_edges):
    ordered_edges.clear()
    nEdges = len(slice_edges)
    ordered_edges.append(slice_edges[0])

    bEdgeReversed = False

    for i in range(0,nEdges):
        iEdge = slice_edges[i]
        v1, v2 = get_edge_vertices(iEdge)
        if not bEdgeReversed:
            vf = v2
        else:
            vf = v1
        for j in range(i+1,nEdges):
            jEdge = slice_edges[j]
            v3, v4 = get_edge_vertices(jEdge)
            if vertices_are_equal(vf,v3,1e-3):
                slice_edges[i+1], slice_edges[j] = slice_edges[j], slice_edges[i+1]
                ordered_edges.append(slice_edges[i+1])
                bEdgeReversed = False
                break
            elif vertices_are_equal(vf,v4,1e-3):
                slice_edges[i+1], slice_edges[j] = slice_edges[j], slice_edges[i+1]
                ordered_edges.append(slice_edges[i+1])
                bEdgeReversed = True
                break



#ordering the edges in correct orientation
def orderedEdges(slice_edges):
    ordered_edges = []
    # Call the corresponding function from gen_htp.py
    EdgesOrdering(slice_edges, ordered_edges)
    return ordered_edges



def OrientLoop(ordered_edges, oriented_edges, loop_orientation):
    # Check if the orientation vector is valid
    # if loop_orientation.X() == 0 and loop_orientation.Y() == 0 and loop_orientation.Z() == 0:
    #     return  # Invalid orientation vector

    # Get the vertices of the ordered edges
    first_vertices = []
    for i in range(0,len(ordered_edges)):
        first_vertex, _ = get_edge_vertices(ordered_edges[i])
        first_point = BRep_Tool.Pnt(first_vertex)
        first_vertices.append(first_point)

    # Close the loop by appending the first vertex again at the end
    first_vertices.append(first_vertices[0])

    # Calculate the area of the loop using the cross product
    area = 0
    origin = gp_Pnt(0, 0, 0)
    for i in range(1, len(first_vertices)):
        point1 = first_vertices[i - 1]
        point2 = first_vertices[i]
        vec1 = gp_Vec(point1, origin)
        vec2 = gp_Vec(point2, origin)
        vec_area = vec1.Crossed(vec2)
        area += vec_area.Z()

    # Determine the orientation of the loop relative to the input direction
    if area * loop_orientation.Z() < 0:
        # Reverse the loop if necessary
        for edge in reversed(ordered_edges):
            oriented_edges.append(edge.Reversed())
    else:
        # Add edges in the original order
        for edge in ordered_edges:
            oriented_edges.append(edge)



def orientedEdges(ord_edges,oriented_edges):
    # oriented_edges = []
    # Add loop orientation if applicable
    dir_loop_orientation = gp_Dir(0, 0, 1)  # Set loop orientation appropriately
    OrientLoop(ord_edges, oriented_edges, dir_loop_orientation)
    return oriented_edges

def toTopoDS_Edge(shape):
    if isinstance(shape, TopoDS_Edge):
        return shape
    elif isinstance(shape, TopoDS_Shape):
        try:
            edge = topods.Edge(shape)
            return edge
        except Exception as e:
            raise ValueError(f"Conversion to TopoDS_Edge failed: {e}")
    else:
        raise TypeError("Cannot convert shape to TopoDS_Edge")

def pointGen(edge, ref_pnt):
    # Ordered points on the sliced edge
    pnts = []      # List to store (X, Y, Z) tuples of points
    pnts_gp = []   # List to store the gp_Pnt objects (geometric points)

    # Check if the input is a valid edge or can be converted to one
    if not isinstance(edge, TopoDS_Edge):
        if isinstance(edge, TopoDS_Shape):
            edge = toTopoDS_Edge(edge)  # Attempt to convert to TopoDS_Edge
        else:
            raise TypeError("The provided object is not a valid TopoDS_Edge or convertible.")

    # Call the corresponding function from gen_htp.py to calculate the start point
    startPnt = gp_Pnt(ref_pnt[0], ref_pnt[1], ref_pnt[2])  # Use gp_Pnt for the reference point
    length = []  # To store the calculated length of the edge

    # Generate points along the edge and store them in `vContPnts` (points as gp_Pnt)
    vContPnts = []
    GeneratePoints(edge, vContPnts, startPnt, length)

    # Process the generated points (vContPnts) to extract their (X, Y, Z) coordinates
    for gp_point in vContPnts:
        x, y, z = round(gp_point.X(), 1), round(gp_point.Y(), 1), round(gp_point.Z(), 1)
        pnts.append((x, y, z))     # Append the tuple (X, Y, Z) to the pnts list
        pnts_gp.append(gp_point)   # Append the gp_Pnt object to the pnts_gp list

    # Return the list of ordered points (pnts) and gp_Pnt objects (pnts_gp)
    return pnts, pnts_gp

def GeneratePoints(aEdge, vContPnts, startPnt, length):
    # Get the vertices of the edge
    first_vertex, last_vertex = get_edge_vertices(aEdge)
    # print(aEdge)
    # Get the geometric points of the vertices
    pnt_f_vertex = BRep_Tool.Pnt(first_vertex)
    pnt_l_vertex = BRep_Tool.Pnt(last_vertex)

    # Create a BRepAdaptor_Curve from the edge
    brep_curve = BRepAdaptor_Curve(aEdge)
    brep_first_param = brep_curve.FirstParameter()
    brep_last_param = brep_curve.LastParameter()

    # Calculate curve length
    c_length = GCPnts_AbscissaPoint.Length(brep_curve)
    length.append(c_length)  # Store the curve length

    # Number of points to generate along the curve
    n_points = 20  # Keep this the same as before

    # Determine the direction based on distance to the start point
    if startPnt.Distance(pnt_f_vertex) < startPnt.Distance(pnt_l_vertex):
        for i in range(0, n_points + 1):
            param = brep_first_param + (brep_last_param - brep_first_param) * (i / n_points)
            pnt = brep_curve.Value(param)
            vContPnts.append(pnt)  # Store gp_Pnt in the vContPnts list
    else:
        for i in range(0, n_points + 1):
            param = brep_last_param - (brep_last_param - brep_first_param) * (i / n_points)
            pnt = brep_curve.Value(param)
            vContPnts.append(pnt)  # Store gp_Pnt in the vContPnts list

    return
#generating normals on ordered points
def normalGen(gSurface, pnts_gp):
    #storing normals of generated points
    normals=[]
    nor_gp=[]
    #iterating over the points on current edge
    for npnt in pnts_gp:
            b, u, v=GeomLib_Tool.Parameters(gSurface, gp_Pnt(npnt.X(), npnt.Y(), npnt.Z()), 1)
            if b==True:
                evaluator=GeomLProp_SLProps(gSurface, u, v, 2, 1e-6)
                normal=evaluator.Normal()
                if normal.XYZ()*(z_dir.XYZ())<0:
                    #reverses the direction of normal
                    normal=-normal
                normal_vec=gp_Vec(normal)
                nor_gp.append(normal_vec)
                normal_points=(normal_vec.X(), normal_vec.Y(), normal_vec.Z())
                normals.append(normal_points)
    #returning list of normals for ordered points       
    return normals, nor_gp

def spiralGen(pnt_slice, prevpnt_slice):
    #storing points on current slice 
    triplepnt=[]
    #storing points on previous slice
    prevtriplepnt=[]
    #storing points on current spiral
    spiralPnts=[]
    #iterating over the list of points on current sliced edge
    for i in range(len(pnt_slice)):
        X=pnt_slice[i].X()
        Y=pnt_slice[i].Y()
        Z=pnt_slice[i].Z()
        triplepnt.append(gp_XYZ(X, Y, Z))
    #iterating over the list of points on previous sliced edge
    for i in range(len(prevpnt_slice)):
        X=prevpnt_slice[i].X()
        Y=prevpnt_slice[i].Y()
        Z=prevpnt_slice[i].Z()
        prevtriplepnt.append(gp_XYZ(X, Y, Z))
    #generating denominator for slice
    deno=0
    for cnt in range(len(pnt_slice)-1):
        pntcurrent=(triplepnt[cnt])
        pntnext=(triplepnt[cnt+1])
        subd=pntcurrent.Subtracted(pntnext)
        deno=deno+subd.Modulus()
    #generating numerator for slice
    num=0
    for cnt in range(len(pnt_slice)):
        if cnt<len(pnt_slice)-1:
            pntcurrent=(triplepnt[cnt])
            pntnext=(triplepnt[cnt+1])
            subd=pntcurrent.Subtracted(pntnext)
            num=num+subd.Modulus()
        s=num/deno
        if len(prevpnt_slice)==len(pnt_slice):
            pntfirst=(triplepnt[cnt]).Multiplied(s)
            pntsecond=(prevtriplepnt[cnt]).Multiplied(1-s)
            sp_pnt=pntfirst.Added(pntsecond)
            spiralPnts.append(sp_pnt)
    #if len(spiralPnts)!=0:
           #spiralPnts.pop()
        
    return spiralPnts

def spnorm(slice_norm, prevslice_norm):
    #storing points on current slice 
    triplepnt=[]
    #storing points on previous slice
    prevtriplepnt=[]
    #storing points on current spiral
    spiralPnts=[]
    #iterating over the list of points on current sliced edge
    for i in range(len(slice_norm)):
        X=slice_norm[i].X()
        Y=slice_norm[i].Y()
        Z=slice_norm[i].Z()
        triplepnt.append(gp_XYZ(X, Y, Z))
    #iterating over the list of points on previous sliced edge
    for i in range(len(prevslice_norm)):
        X=prevslice_norm[i].X()
        Y=prevslice_norm[i].Y()
        Z=prevslice_norm[i].Z()
        prevtriplepnt.append(gp_XYZ(X, Y, Z))
    #generating denominator for slice
    deno=0
    for cnt in range(len(slice_norm)-1):
        pntcurrent=(triplepnt[cnt])
        pntnext=(triplepnt[cnt+1])
        subd=pntcurrent.Subtracted(pntnext)
        deno=deno+subd.Modulus()
    #generating numerator for slice
    num=0
    for cnt in range(len(slice_norm)):
        if cnt<len(slice_norm)-1:
            pntcurrent=(triplepnt[cnt])
            pntnext=(triplepnt[cnt+1])
            subd=pntcurrent.Subtracted(pntnext)
            num=num+subd.Modulus()
        s=num/deno
        if len(prevslice_norm)==len(slice_norm):
            pntfirst=(triplepnt[cnt]).Multiplied(s)
            pntsecond=(prevtriplepnt[cnt]).Multiplied(1-s)
            sp_pnt=pntfirst.Added(pntsecond)
            spiralPnts.append(sp_pnt)
    #if len(spiralPnts)!=0:
           #spiralPnts.pop()
        
    return spiralPnts

#address of file storing toolpath
f_N3=''
def gen_toolpath(f_N1, f_N2, TD1, Feed, cnc, gen_type, folder):
    # global spir_folder, cont_folder
    #tool diameter (taken as a input from user)
    TD1=float(TD1)
    #feedrate (taken as a input from user)
    Feed=int(Feed)
    #tool radius
    R1=TD1/2
    #removing the unnecessary characters from the the files storing points and normal
    #inorder to perform required calcultions 
    def clean_and_loadtxt(file_path):
        cleaned_data = []
        with open(file_path, 'r') as file:
            for line in file:
                #Replacing comas with blank
                cleaned_line = line.replace(',', ' ').strip().split()
                cleaned_row = []
                for value in cleaned_line:
                    try:
                        cleaned_row.append(float(value))
                    except ValueError:
                        print(f"Warning: Skipping invalid value '{value}'.")
                if cleaned_row:
                    cleaned_data.append(cleaned_row)
        #returing a numpy array
        return np.array(cleaned_data)
    #loading points to perform calculations
    C=clean_and_loadtxt(f_N1)
    #loading normal to perform calculations
    nC=clean_and_loadtxt(f_N2)
    #checking if they have same number of data
    if C.shape != nC.shape:
        raise ValueError("S and nS must have the same shape")
    
    TCS = C+nC*R1

    TTS = TCS.copy()

    TTS[:, 2] = TCS[:, 2]-R1

    L = TTS.shape[0]

    LNO = 4
    
    file_name=gen_type+".mpf"
    #file path for stotring toolpath(change the address to the address of your pc while locally hosting)
    file_path=f"{folder}/"+file_name
    global f_N3
    f_N3=file_path

    with open(file_path, 'w') as fid:
        #storing gcodes in the file in the format accepted by fanuc type controllers
        if cnc=='Fanuc':
            fid.write('N1 G54 F2500;\n')
            fid.write('N2 G00 Z50;\n')
            fid.write('N3 G64;\n')
            fid.write(f'N4  G01   X{TTS[0, 0]:5.5f}   Y{TTS[0, 1]:5.5f}   F{Feed:5.5f};\n')

            for i in range(L):
                fid.write(f'N{LNO + i + 1}  G01   X{TTS[i, 0]:5.5f}   Y{TTS[i, 1]:5.5f}   Z{TTS[i, 2]:5.5f};\n')

            fid.write(f'N{LNO + L + 1}  G01  Z50.00000;\n')
            fid.write(f'N{LNO + L + 2}  M30;\n')
        #storing gcodes in the file in the format accepted by siemens controllers
        elif cnc=='Siemens':
            fid.write('N1 G54 F2500\n')
            fid.write('N2 G00 Z=50\n')
            fid.write('N3 G64\n')
            fid.write(f'N4  G01   X={TTS[0, 0]:5.5f}   Y={TTS[0, 1]:5.5f}   F{Feed:5.5f}\n')

            for i in range(L):
                fid.write(f'N{LNO + i + 1}  G01   X={TTS[i, 0]:5.5f}   Y={TTS[i, 1]:5.5f}   Z={TTS[i, 2]:5.5f}  F{Feed:5.5f}\n')

            fid.write(f'N{LNO + L + 1}  G01  Z=50.00000  F{Feed:5.5f}\n')
            fid.write(f'N{LNO + L + 2}  M30\n')

#generating plots of sliced and spiral geomerty
def plot(txt, html, plotTitle):
    def clean_line(line):
        #Removing any unwanted characters, like commas
        clean_line = line.replace(',', '').strip()
        #Spliting the cleaned line into individual string numbers
        string_numbers = clean_line.split()
        #Converting the string numbers to floats
        float_numbers = [float(num) for num in string_numbers]
        return float_numbers
    
    
    #Loading the data from the file
    file_path = txt
    data1 = []

    with open(file_path, 'r') as file:
        for line in file:
            try:
                #Cleaning and converting each line to a list of floats
                data1.append(clean_line(line))
            except ValueError as e:
                print(f"Error converting line to floats: {line}")
                print(f"Error message: {e}")

    #Converting the list of lists to a NumPy array if it's not empty
    if data1:
        s = np.array(data1)
        #Extracting x, y, z coordinates
        x = s[:, 0]
        y = s[:, 1]
        z = s[:, 2]
        #Createing a 3D plot using Plotly
        fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z, mode='lines', marker=dict(size=2))])
        fig.update_layout(scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ), title= plotTitle
        )
        #Saving the plot as an HTML file
        output_file = html
        fig.write_html(output_file)
        print(f"3D plot saved to {output_file}")
    else:
        print("No data to plot.")

    return s

#simulating the toolpath
def simulate(s, html1, plotTitle):
    frames = []
    for i in range(1, len(s) + 1):
        frame = go.Frame(
            data=[
                go.Scatter3d(
                    x=s[:i, 0],
                    y=s[:i, 1],
                    z=s[:i, 2],
                    mode='lines',
                    line=dict(color='blue')
                )
            ],
            name=f'frame{i}'
        )
        frames.append(frame)

    #Creating Plotly figure
    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=[s[0, 0]],
                y=[s[0, 1]],
                z=[s[0, 2]],
                mode='lines',
                #marker=dict(size=5, color='blue'),
                line=dict(color='blue')
            )
        ],
        layout=go.Layout(
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Z'
            ),
            title=plotTitle,
            updatemenus=[{
                'type': 'buttons',
                'buttons': [{
                    'label': 'Play',
                    'method': 'animate',
                    'args': [None, {'frame': {'duration': 1, 'redraw': True}, 'mode': 'immediate', 'transition': {'duration': 0}, 'fromcurrent': True}]
                }, {
                    'label': 'Pause',
                    'method': 'animate',
                    'args': [[None], {'frame': {'duration': 0, 'redraw': False}, 'mode': 'immediate', 'transition': {'duration': 0}}]
                }]
            }]
        ),
        frames=frames
    )
    #storing plot as html file
    output_file = html1
    fig.write_html(output_file)


#address of files storing points
f_N1=''
#address of files storing normals
f_N2=''
#address of files storing spiral points
f_N4=''
#address of files storing spiral normals
f_N5=''
# #reference point for generating points to check whether any repetative points are there
# ref_pnt1=[-1000,0,0]
# #referance points for generating point to be stored in a file
# ref_pnt2=[-1000,0,0]
# # #reference point to obtain first edge
# st_pnt=gp_Pnt(-1000, 0, 0)
# #direction vector
z_dir=gp_Dir(0,0,1)
#main function (calling all the function here)

def process_step_file(step_path,contour_folder, spiral_folder):
    #calling global variables
    # global spir_folder, cont_folder
    global z_dir
    global st_pnt
    global ref_pnt1
    global ref_pnt2
    
    
    file_name1="pntContour.txt"
    #file path for stotring points(change the address to the address of your pc while locally hosting)
    file_path1= os.path.join(contour_folder, file_name1)
    global f_N1
    f_N1=file_path1
    #Opens a file to store points  
    f1=open(file_path1,"w")

    file_name2="nContour.txt"
    #file path for stotring normal(change the address to the address of your pc while locally hosting)
    file_path2=os.path.join(contour_folder, file_name2)
    global f_N2
    f_N2=file_path2
    #Opens a file to store normals
    f2=open(file_path2,"w")

    file_name4="pntspiral.txt"
    #file path for stotring spiral points(change the address to the address of your pc while locally hosting)
    file_path4=os.path.join(spiral_folder, file_name4)
    global f_N4
    f_N4=file_path4
    #opens a file to store spiral points
    f4=open(file_path4,'w')

    file_name5="nspiral.txt"
    #file path for stotring spiral normal(change the address to the address of your pc while locally hosting)
    file_path5=os.path.join(spiral_folder, file_name5)
    global f_N5
    f_N5=file_path5
    #opens a file to store spiral normals
    f5=open(file_path5,'w')
    
    step_reader = STEPControl_Reader()
    step_reader.ReadFile(step_path)
    step_reader.TransferRoots()
    shape = step_reader.OneShape()
    def get_shape_reference_points(shape):
        # Compute the bounding box of the shape to get unique reference points
        bbox = Bnd_Box()
        brepbndlib_Add(shape, bbox)
        
        # Get the center of the bounding box as a dynamic reference point
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        center_z = (zmin + zmax) / 2

        # Set reference points dynamically based on the bounding box
        ref_pnt1 = [xmin, ymin, zmin]   # Use min corner for checking duplicates
        ref_pnt2 = [xmax, ymax, zmax]   # Use max corner for storing points
        st_pnt = gp_Pnt(xmax, ymax, zmax)  # Center of the shape for closest edge

        return ref_pnt1, ref_pnt2, st_pnt
    # Usage example with each shape
    ref_pnt1, ref_pnt2, st_pnt = get_shape_reference_points(shape)

    #creating a box around the geometry to get the height
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    z_min, z_max = box.CornerMin().Z(), box.CornerMax().Z()
    l=z_max-z_min
    dz=0.5
    n=int(l/dz)
    
    #storing all points on slice, normals and points on spiral
    all_pnt=[]
    all_normal=[]
    all_spiralpnt=[]
    all_spnormal=[]

    #storing points on current slice
    pnt_slice=[]
    #storing points on previous slice
    prevpnt_slice=[]
    norm_slice=[]
    prevnorm_slice=[]
    #slicing with the incremental depth of dz
    for i in range(1,n):
        if i==1:
            z=z_max-0.1
        else:
            z=z-dz
        
        #defining slicing plane
        plane = gp_Pln(gp_Ax3(gp_Pnt(0, 0, z), z_dir))
        section = BRepAlgoAPI_Section(shape, plane, False)
        section.ComputePCurveOn1(True)
        section.Approximation(True)
        section.Build()
        if not section.IsDone():
            return 'Slicing failed'
    
        exp = TopExp_Explorer(section.Shape(), TopAbs_EDGE)
        #stors edges as list
        edges=[]
        BRepMesh_IncrementalMesh(shape, 0.1)
        while exp.More():
            #find no.of times while occurs =>no.of edges
            edge = topods.Edge(exp.Current())
            edges.append(edge)
            exp.Next()
        #getting first edge
        slice_edges=stPnt(st_pnt, edges)
        #getting edges ordered
        ordered_edges=orderedEdges(slice_edges)
        #getting edges oriented to have same direction of loop area
        oriented_edges = []
        oriented_edges=orientedEdges(ordered_edges,oriented_edges)
        norm_slice.clear()
        #storing list of points on edges on a single slice
        currpnt=[]
        edge_count1=0
        for edge in oriented_edges:
            pnts, pnts_gp = pointGen(edge, ref_pnt1)
            ref_pnt1=pnts[len(pnts)-1]
            currpnt.append(pnts)
        #storing the repeated edges
        pedge=[]
        #iterating over the list of lists of points on single edge
        for l in range(len(currpnt)):
            for m in range(l+1, len(currpnt)):
                #iterating over the list of points on single edge
                for lp in range(1,len(currpnt[0])-1):
                    #checking if any two lists has same points
                    if currpnt[l][lp]==currpnt[m][len(currpnt[0])-1-lp] or currpnt[l][lp]==currpnt[m][lp]:
                        f=0
                        for ie in pedge:
                            if ie==m:
                                f=1
                        if f==0:
                            #appending the repeated edge 
                            pedge.append(m)              
        #iterating the repeated edges to remove the from the list
        redge=[]
        for ed in pedge:
            redge.append(oriented_edges[ed])
        for redg in redge:
            oriented_edges.remove(redg)
            
        #storing current slice points
        vconstpnt=[]
        #iterating over the modified list of edges to genrate points
        for e in range(len(oriented_edges)): 
            #getting ordered points
            pnts1, pnts_gp1 = pointGen(oriented_edges[e], ref_pnt2)
            ref_pnt2=pnts1[len(pnts1)-1]
            for p in pnts_gp1:
                vconstpnt.append(p)

            #getting face containing that point
            face=TopoDS_Shape()
            bAncestor1 = BRepAlgoAPI_Section.HasAncestorFaceOn1(section, oriented_edges[e], face)
            #checking if edge lies on this face or not
            if bAncestor1==True:
                gSurface = BRep_Tool.Surface(topods.Face(face))
                #getting normal vectors on generated points
                normals, nor_gp=normalGen(gSurface, pnts_gp1)
            for normal in normals:
                all_normal.append(normal)
            for norm in nor_gp:
                norm_slice.append(norm)

        #appending ordered points on current slice to list of all points
        for v in vconstpnt:
            all_pnt.append(v)
        
        #deleting all previous elements
        pnt_slice.clear()
        #appending points on current slice
        pnt_slice=vconstpnt
        #generating spiral points
        spiralPnts=spiralGen(pnt_slice, prevpnt_slice)
        for sp in (spiralPnts):
            all_spiralpnt.append(sp)
            
        prevpnt_slice.clear()
        #appending current slice points to previous slice points
        for spnt in (pnt_slice):
            prevpnt_slice.append(spnt)

        spnor=spnorm(norm_slice, prevnorm_slice)
        for no in spnor:
            all_spnormal.append(no)
        prevnorm_slice.clear()
        for np in norm_slice:
            prevnorm_slice.append(np)
          
    #appending all the complete list to  respective files
    #slice points
    for pcontour in all_pnt:
        pnt=(pcontour.X(), pcontour.Y(), pcontour.Z())
        point_to_append1=str(pnt).replace("(","").replace(")","")
        f1.write(point_to_append1+"\n")
    #normals
    for ncontour in all_normal:
        normal_str=str(ncontour).replace("(","").replace(")","")
        f2.write(normal_str+"\n")
    #spiral points
    for sp_pnt in all_spiralpnt:
        spiral=(sp_pnt.X(), sp_pnt.Y(), sp_pnt.Z())
        str_sp=str(spiral).replace("(","").replace(")","")
        f4.write(str_sp+'\n')
    #append spiral noirmals to file
    for sp_nor in all_spnormal:
        gp_nor=(sp_nor.X(), sp_nor.Y(), sp_nor.Z())
        str_nsp=str(gp_nor).replace("(","").replace(")","")
        f5.write(str_nsp+"\n")

    
    f1.close()
    f2.close()
    f4.close()
    f5.close()
    return "Points and normals are successfully generated!"


# Function to zip a folder
def zip_folder(folder_path):
    memory_file = io.BytesIO()
    with zipfile.ZipFile(memory_file, 'w', zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in os.walk(folder_path):
            for file in files:
                zf.write(os.path.join(root, file),
                         os.path.relpath(os.path.join(root, file), 
                                         os.path.join(folder_path, '..')))
    memory_file.seek(0)
    return memory_file

@app.route('/feedback', methods=['GET'])
def feedback_form():
    return render_template('feedback.html')
@app.route('/comment', methods=['POST'])
def feedback():
    global name
    comment = request.form['comment']
    
    # Find the user by their name (assuming it's unique)
    user = User.query.filter_by(first_name=name).first()
    if user:
        # Create a new Feedback entry with the user's name
        new_feedback = Feedback(user_id=user.id, user_name=user.first_name + " " + user.last_name, comment=comment)
        db.session.add(new_feedback)
        db.session.commit()
        # flash("Feedback Submitted Successfully")
        message = f'Welcome {session["user_name"]}'
        return render_template('index.html', message=message)
    else:
        return "User not found", 404




#downloads file containing points
@app.route('/download1')
def download_file1():
    file_location = zip_folder(f"{contour_folder}") 
    return send_file(file_location, as_attachment=True, download_name='Contour_'+str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))+'.zip')

#downloads flie containing normals
@app.route('/download2')
def download_file2():
    file_location = zip_folder(f"{spiral_folder}") 
    return send_file(file_location, as_attachment=True, download_name='Spiral_'+str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))+'.zip')

@app.route('/simul1')
def simulate_contour():
    ht1=BASE_PATH + "static/simulContour.html"
    simulate(scontour, ht1, 'Contour Trajectory')
    return render_template('simulate1.html')

@app.route('/simul2')
def simulate_spiral():
    ht2=BASE_PATH + "static/simulSpiral.html"
    simulate(sspiral, ht2, 'Spiral Trajectory')
    return render_template('simulate2.html')

@app.route('/view')
def exit_simul():
    return render_template('view.html')

@app.route('/view/<filename>')
def view_file(filename):
    return render_template('view.html', filename=filename)

@app.route('/uploads/<filename>')
def uploaded_file(filename):
    return send_from_directory(app.config['UPLOAD_FOLDER'], filename)

if __name__ == '__main__':
    #creating the directories if already not created
    if not os.path.exists(UPLOAD_FOLDER):
        os.makedirs(UPLOAD_FOLDER)
    app.run(host='0.0.0.0', port=5000)
