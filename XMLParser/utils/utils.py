import errno

from lxml import etree
from xml.etree import ElementTree
import os
def printxml(t, fn):
    """Print information from a dict into an xml file."""
    etree.ElementTree(t).write(
        fn, pretty_print=True, encoding='UTF-8', xml_declaration=True)


def makexml(name, nsl):
    """Create an xml file."""
    xsi = "http://www.w3.org/2001/XMLSchema-instance"
    ns = {"xsi": xsi}
    attr = {"{%s}noNamespaceSchemaLocation" % xsi: nsl}
    t = etree.Element(name, attrib=attr, nsmap=ns)
    return t


def ensure_dir(path):
    """Ensure that the directory specified exists, and if not, create it."""
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
    return path

