"""Module that contains a number of constraint object classes
that can be used to add constraint to the moveit planning
scene. It was created since the moveit_commander doesn't yet
contain a addCollisionObjects method. As a result using the
standard collision_object_msgs and shape_msgs was not ideal."""

# ROS messages and services
from geometry_msgs.msg import PoseStamped


#################################################
# Constraint classes ############################
#################################################
class Box(object):
    """Moveit Box collision object class.

    Attributes
    ------------
        type : :py:obj:`str`
            Collision object type.
        name : :py:obj:`str`
            The name of the object.
        size : :py:obj:`tuple`
            The size of the box specified as (x, y, z).
        pose : :py:obj:`!geometry_msgs.PoseStamed`
            The pose of the collision object.
    """

    def __init__(
        self,
        name,
        size_x=1.0,
        size_y=1.0,
        size_z=1.0,
        x_pos=0.0,
        y_pos=0.0,
        z_pos=0.0,
        x_rot=0.0,
        y_rot=0.0,
        z_rot=0.0,
        w_rot=1.0,
        reference_frame="world",
    ):
        """

        Parameters
        ----------
        name : :py:obj:`str`
            The name of the object.
        size_x : :py:obj:`float`, optional
            The x-dimensions size of the box, by default 1.0
        size_y : :py:obj:`float`, optional
            The y-dimensions size of the box, by default 1.0
        size_z : :py:obj:`float`, optional
            The z-dimensions size of the box, by default 1.0
        x_pos : :py:obj:`float`, optional
            The x position in link_name frame, by default 0.0.
        y_pos : :py:obj:`float`, optional
            The y position in link_name frame, by default 0.0.
        z_pos : :py:obj:`float`, optional
            The z position in link_name frame, by default 0.0.
        x_rot : :py:obj:`float`, optional
            The x orientation relative to the link_name frame, by default 0.0.
        y_rot : :py:obj:`float`, optional
            The y orientation relative to the link_name frame, by default 0.0.
        z_rot : :py:obj:`float`, optional
            The z orientation relative to the link_name frame, by default 0.0.
        w_rot : :py:obj:`float`, optional
            The w orientation relative to the link_name frame, by default 1.0.
        reference_frame : :py:obj:`str`, optional
            The frame in which the pose is expressed, by default world.
        """

        # Set member variables
        self.type = "box"
        self.name = name
        self.pose = PoseStamped()
        self.pose.header.frame_id = reference_frame
        self.pose.pose.position.x = x_pos
        self.pose.pose.position.y = y_pos
        self.pose.pose.position.z = z_pos
        self.pose.pose.orientation.x = x_rot
        self.pose.pose.orientation.y = y_rot
        self.pose.pose.orientation.z = z_rot
        self.pose.pose.orientation.w = w_rot
        self._size = (size_y, size_y, size_z)

    @property
    def size(self):
        """Get the size of the collision object."""
        return self._size

    @size.setter
    def size(self, value):
        if not isinstance(value, tuple) or len(value) != 3:  # Validate if tuple
            raise Exception("Size should be a tuple of length 3 (x, y, z).")
        self._size = value


class Plane(object):
    """Moveit Cube collision object class.

    .. note::
        This class uses the plane equation Ax+Bx+Cz + D = 0
        to construct the plane. In this A,B,C represent the
        the coordinates of the plane normal (Orientation) and
        D the offset to the normal.

    Attributes
    ------------
        type : :py:obj:`str`
            Collision object type.
        name : :py:obj:`str`
            The name of the object.
        normal : :py:obj:`list`
            Plane normal specified as (x, y, z).
        offset : :py:obj:`float`
            Plane offset relative to the normal.
        pose : :py:obj:`!geometry_msgs.PoseStamed`
            The pose of the collision object.
    """

    def __init__(
        self,
        name,
        normal_x=0.0,
        normal_y=0.0,
        normal_z=1.0,
        offset=0.0,
        x_pos=0.0,
        y_pos=0.0,
        z_pos=0.0,
        x_rot=0.0,
        y_rot=0.0,
        z_rot=0.0,
        w_rot=1.0,
        reference_frame="world",
    ):
        """

        Parameters
        ----------
        name : :py:obj:`str`
            The name of the object.
        normal_x : :py:obj:`float`, optional
            The normal vector x coordinate, by default 0.0.
        normal_y : :py:obj:`float`, optional
            The normal vector y coordinate, by default 0.0.
        normal_z : :py:obj:`float`, optional
            The normal vector z coordinate, by default 1.0.
        offset : py:obj:`float`, optional
            Plane offset relative to the normal, by default 0.0
        x_pos : :py:obj:`float`, optional
            The x position in link_name frame, by default 0.0.
        y_pos : :py:obj:`float`, optional
            The y position in link_name frame, by default 0.0.
        z_pos : :py:obj:`float`, optional
            The z position in link_name frame, by default 0.0.
        x_rot : :py:obj:`float`, optional
            The x orientation relative to the link_name frame, by default 0.0.
        y_rot : :py:obj:`float`, optional
            The y orientation relative to the link_name frame, by default 0.0.
        z_rot : :py:obj:`float`, optional
            The z orientation relative to the link_name frame, by default 0.0.
        w_rot : :py:obj:`float`, optional
            The w orientation relative to the link_name frame, by default 1.0.
        reference_frame : :py:obj:`str`, optional
            The frame in which the pose is expressed, by default world.
        """

        # Set member variables
        self.type = "plane"
        self.name = name
        self.offset = offset
        self.reference_frame = reference_frame
        normal_tmp = (float(normal_x), float(normal_y), float(normal_z))
        self._normal = tuple(
            ti / sum(normal_tmp) for ti in normal_tmp
        )  # Normalize tuple
        self.pose = PoseStamped()
        self.pose.header.frame_id = reference_frame
        self.pose.pose.position.x = x_pos
        self.pose.pose.position.y = y_pos
        self.pose.pose.position.z = z_pos
        self.pose.pose.orientation.x = x_rot
        self.pose.pose.orientation.y = y_rot
        self.pose.pose.orientation.z = z_rot
        self.pose.pose.orientation.w = w_rot

    @property
    def normal(self):
        """The plane normal."""
        return self._normal

    @normal.setter
    def normal(self, value):
        if not isinstance(value, tuple) or len(value) != 3:  # Validate if tuple
            raise Exception("Size should be a tuple of length 3 (x, y, z).")
        normal_tmp = tuple(float(ti) for ti in value)  # Create float
        self._normal = tuple(
            ti / sum(normal_tmp) for ti in normal_tmp
        )  # Normalize tuple


class Cylinder(object):
    """Moveit cylinder collision object class.

    Attributes
    ------------
        type : :py:obj:`str`
            Collision object type.
        name : :py:obj:`str`
            The name of the object.
        height : :py:obj:`float`
            The height of the cylinder.
        radius : :py:obj:`float`
            The radius of the cylinder.
        x_pos : :py:obj:`float`
            The x position in link_name frame.
        y_pos : :py:obj:`float`
            The y position in link_name frame.
        z_pos : :py:obj:`float`
            The z position in link_name frame.
        x_rot : :py:obj:`float`
            The x orientation relative to the link_name frame.
        y_rot : :py:obj:`float`
            The y orientation relative to the link_name frame.
        z_rot : :py:obj:`float`
            The z orientation relative to the link_name frame.
        w_rot : :py:obj:`float`
            The w orientation relative to the link_name frame.
        pose : :py:obj:`!geometry_msgs.PoseStamped`
            The object pose.
    """

    def __init__(
        self,
        name,
        height=1.0,
        radius=1.0,
        x_pos=0.0,
        y_pos=0.0,
        z_pos=0.0,
        x_rot=0.0,
        y_rot=0.0,
        z_rot=0.0,
        w_rot=1.0,
        reference_frame="world",
    ):
        """

        Parameters
        ----------
        name : :py:obj:`str`
            The name of the object.
        height : :py:obj:`float`, optional
            The height of the cylinder, by default 1.0
        radius : :py:obj:`float`, optional
            The radius of the cylinder, by default 1.0.
        x_pos : :py:obj:`float`, optional
            The x position in link_name frame, by default 0.0.
        y_pos : :py:obj:`float`, optional
            The y position in link_name frame, by default 0.0.
        z_pos : :py:obj:`float`, optional
            The z position in link_name frame, by default 0.0.
        x_rot : :py:obj:`float`, optional
            The x orientation relative to the link_name frame, by default 0.0.
        y_rot : :py:obj:`float`, optional
            The y orientation relative to the link_name frame, by default 0.0.
        z_rot : :py:obj:`float`, optional
            The z orientation relative to the link_name frame, by default 0.0.
        w_rot : :py:obj:`float`, optional
            The w orientation relative to the link_name frame, by default 1.0.
        reference_frame : :py:obj:`str`, optional
            The frame in which the pose is expressed, by default world.
        """

        # Set member variables
        self.type = "cylinder"
        self.name = name
        self.height = height
        self.radius = radius
        self.pose = PoseStamped()
        self.pose.header.frame_id = reference_frame
        self.pose.pose.position.x = x_pos
        self.pose.pose.position.y = y_pos
        self.pose.pose.position.z = z_pos
        self.pose.pose.orientation.x = x_rot
        self.pose.pose.orientation.y = y_rot
        self.pose.pose.orientation.z = z_rot
        self.pose.pose.orientation.w = w_rot


class Sphere(object):
    """Moveit sphere collision object class.

    Attributes
    ------------
        type : :py:obj:`str`
            Collision object type.
        name : :py:obj:`str`
            The name of the object.
        radius : :py:obj:`float`
            The radius of the sphere.
        x_pos : :py:obj:`float`
            The x position in link_name frame.
        y_pos : :py:obj:`float`
            The y position in link_name frame.
        z_pos : :py:obj:`float`
            The z position in link_name frame.
        x_rot : :py:obj:`float`
            The x orientation relative to the link_name frame.
        y_rot : :py:obj:`float`
            The y orientation relative to the link_name frame.
        z_rot : :py:obj:`float`
            The z orientation relative to the link_name frame.
        w_rot : :py:obj:`float`
            The w orientation relative to the link_name frame.
        pose : :py:obj:`!geometry_msgs.PoseStamped`
            The object pose.
    """

    def __init__(
        self,
        name,
        radius=1.0,
        x_pos=0.0,
        y_pos=0.0,
        z_pos=0.0,
        x_rot=0.0,
        y_rot=0.0,
        z_rot=0.0,
        w_rot=1.0,
        reference_frame="world",
    ):
        """

        Parameters
        ----------
        name : :py:obj:`str`
            The name of the object.
        radius : :py:obj:`float`, optional
            The radius of the sphere, by default 1.0.
        x_pos : :py:obj:`float`, optional
            The x position in link_name frame, by default 0.0.
        y_pos : :py:obj:`float`, optional
            The y position in link_name frame, by default 0.0.
        z_pos : :py:obj:`float`, optional
            The z position in link_name frame, by default 0.0.
        x_rot : :py:obj:`float`, optional
            The x orientation relative to the link_name frame, by default 0.0.
        y_rot : :py:obj:`float`, optional
            The y orientation relative to the link_name frame, by default 0.0.
        z_rot : :py:obj:`float`, optional
            The z orientation relative to the link_name frame, by default 0.0.
        w_rot : :py:obj:`float`, optional
            The w orientation relative to the link_name frame, by default 1.0.
        reference_frame : :py:obj:`str`, optional
            The frame in which the pose is expressed, by default world.
        """

        # Set member variables
        self.type = "sphere"
        self.name = name
        self.radius = radius
        self.pose = PoseStamped()
        self.pose.header.frame_id = reference_frame
        self.pose.pose.position.x = x_pos
        self.pose.pose.position.y = y_pos
        self.pose.pose.position.z = z_pos
        self.pose.pose.orientation.x = x_rot
        self.pose.pose.orientation.y = y_rot
        self.pose.pose.orientation.z = z_rot
        self.pose.pose.orientation.w = w_rot


class Mesh(object):
    """Moveit mesh collision object class.

    Attributes
    ------------
        type : :py:obj:`str`
            Collision object type.
        name : :py:obj:`str`
            The name of the object.
        file_name : :py:obj:`str`
            The location of the mesh file.
        x_pos : :py:obj:`float`
            The x position in link_name frame.
        y_pos : :py:obj:`float`
            The y position in link_name frame.
        z_pos : :py:obj:`float`
            The z position in link_name frame.
        x_rot : :py:obj:`float`
            The x orientation relative to the link_name frame.
        y_rot : :py:obj:`float`
            The y orientation relative to the link_name frame.
        z_rot : :py:obj:`float`
            The z orientation relative to the link_name frame.
        w_rot : :py:obj:`float`
            The w orientation relative to the link_name frame.
        pose : :py:obj:`!geometry_msgs.PoseStamped`
            The object pose.
    """

    def __init__(
        self,
        name,
        file_name,
        x_pos=0.0,
        y_pos=0.0,
        z_pos=0.0,
        x_rot=0.0,
        y_rot=0.0,
        z_rot=0.0,
        w_rot=1.0,
        reference_frame="world",
    ):
        """

        Parameters
        ----------
        name : :py:obj:`str`
            The name of the object
        file_name : :py:obj:`str`
            The location of the mesh file.
        x_pos : :py:obj:`float`, optional
            The x position in link_name frame, by default 0.0.
        y_pos : :py:obj:`float`, optional
            The y position in link_name frame, by default 0.0.
        z_pos : :py:obj:`float`, optional
            The z position in link_name frame, by default 0.0.
        x_rot : :py:obj:`float`, optional
            The x orientation relative to the link_name frame, by default 0.0.
        y_rot : :py:obj:`float`, optional
            The y orientation relative to the link_name frame, by default 0.0.
        z_rot : :py:obj:`float`, optional
            The z orientation relative to the link_name frame, by default 0.0.
        w_rot : :py:obj:`float`, optional
            The w orientation relative to the link_name frame, by default 1.0.
        reference_frame : :py:obj:`str`, optional
            The frame in which the pose is expressed, by default world.
        """

        # Set member variables
        self.type = "mesh"
        self.name = name
        self.file_name = file_name
        self.pose = PoseStamped()
        self.pose.header.frame_id = reference_frame
        self.pose.pose.position.x = x_pos
        self.pose.pose.position.y = y_pos
        self.pose.pose.position.z = z_pos
        self.pose.pose.orientation.x = x_rot
        self.pose.pose.orientation.y = y_rot
        self.pose.pose.orientation.z = z_rot
        self.pose.pose.orientation.w = w_rot
