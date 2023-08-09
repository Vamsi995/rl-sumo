import os
import subprocess
import tempfile
import time
from copy import deepcopy

from lxml import etree
from xml.etree import ElementTree

from XMLParser.utils.utils import makexml, ensure_dir, printxml

E = etree.Element
# Number of retries on accessing the .net.xml file before giving up
RETRIES_ON_ERROR = 10
# number of seconds to wait before trying to access the .net.xml file again
WAIT_ON_ERROR = 1


def _flow(name, vtype, route, **kwargs):
    return E('flow', id=name, route=route, type=vtype, **kwargs)


def _inputs(net=None, rou=None, add=None, gui=None):
    inp = E("input")
    inp.append(E("net-file", value=net))
    inp.append(E("route-files", value=rou))
    inp.append(E("additional-files", value=add))
    inp.append(E("gui-settings-file", value=gui))
    return inp


class TraCINetwork:

    def __init__(self):
        self.net_path = os.path.join(tempfile.gettempdir(), 'debug/net/')
        self.cfg_path = os.path.join(tempfile.gettempdir(), 'debug/cfg/')

        ensure_dir('%s' % self.net_path)
        ensure_dir('%s' % self.cfg_path)

        # variables to be defined during network generation
        self.network = None
        self.nodfn = None
        self.edgfn = None
        self.typfn = None
        self.cfgfn = None
        self.netfn = None
        self.confn = None
        self.roufn = None
        self.addfn = None
        self.sumfn = None
        self.guifn = None
        self._edges = None
        self._connections = None
        self._edge_list = None
        self._junction_list = None
        self.__max_speed = None
        self.__length = None  # total length
        self.__non_internal_length = None  # total length of non-internal edges
        self.rts = None
        self.cfg = None

    def generate_network(self, network):
        self.network = network

        # names of the soon-to-be-generated xml and sumo config files
        self.nodfn = '%s.nod.xml' % self.network.name
        self.edgfn = '%s.edg.xml' % self.network.name
        self.typfn = '%s.typ.xml' % self.network.name
        self.cfgfn = '%s.netccfg' % self.network.name
        self.netfn = '%s.net.xml' % self.network.name
        self.confn = '%s.con.xml' % self.network.name
        self.roufn = '%s.rou.xml' % self.network.name
        self.addfn = '%s.add.xml' % self.network.name
        self.sumfn = '%s.sumo.cfg' % self.network.name
        self.guifn = '%s.gui.cfg' % self.network.name

        self._edges, self._connections = self.generate_net(
            self.network.net_params,
            self.network.traffic_lights,
            self.network.nodes,
            self.network.edges,
            self.network.types,
            connections=None
        )
        self._edge_list = [
            edge_id for edge_id in self._edges.keys() if edge_id[0] != ':'
        ]
        self._junction_list = list(
            set(self._edges.keys()) - set(self._edge_list))

        # maximum achievable speed on any edge in the network
        self.__max_speed = max(
            self.speed_limit(edge) for edge in self.get_edge_list())

        # length of the network, or the portion of the network in
        # which cars are meant to be distributed
        self.__non_internal_length = sum(
            self.edge_length(edge_id) for edge_id in self.get_edge_list()
        )

        # parameters to be specified under each unique subclass's
        # __init__ function
        self.edgestarts = self.network.edge_starts

        # if no edge_starts are specified, generate default values to be used
        # by the "get_x" method
        if self.edgestarts is None:
            length = 0
            self.edgestarts = []
            for edge_id in sorted(self._edge_list):
                # the current edge starts where the last edge ended
                self.edgestarts.append((edge_id, length))
                # increment the total length of the network with the length of
                # the current edge
                length += self._edges[edge_id]['length']

        # these optional parameters need only be used if "no-internal-links"
        # is set to "false" while calling sumo's netconvert function
        self.internal_edgestarts = self.network.internal_edge_starts
        self.internal_edgestarts_dict = dict(self.internal_edgestarts)

        # total_edgestarts and total_edgestarts_dict contain all of the above
        # edges, with the former being ordered by position
        self.total_edgestarts = self.edgestarts + self.internal_edgestarts
        self.total_edgestarts.sort(key=lambda tup: tup[1])

        self.total_edgestarts_dict = dict(self.total_edgestarts)

        self.__length = sum(
            self._edges[edge_id]['length'] for edge_id in self._edges
        )

        if self.network.routes is None:
            print("No routes specified, defaulting to single edge routes.")
            self.network.routes = {edge: [edge] for edge in self._edge_list}

        # specify routes vehicles can take  # TODO: move into a method
        self.rts = self.network.routes

        # create the sumo configuration files
        cfg_name = self.generate_cfg(self.network.net_params,
                                     self.network.traffic_lights,
                                     self.network.routes)

        # specify the location of the sumo configuration file
        self.cfg = self.cfg_path + cfg_name

    def generate_net(self, net_params,
                     traffic_lights,
                     nodes,
                     edges,
                     types=None,
                     connections=None):
        tl_ids = list(traffic_lights.get_properties().keys())
        for n_id in tl_ids:
            indx = next(i for i, nd in enumerate(nodes) if nd['id'] == n_id)
            nodes[indx]['type'] = 'traffic_light'

        # for nodes that have traffic lights that haven't been added
        for node in nodes:
            if node['id'] not in tl_ids \
                    and node.get('type', None) == 'traffic_light':
                traffic_lights.add(node['id'])

            # modify the x and y values to be strings
            node['x'] = str(node['x'])
            node['y'] = str(node['y'])
            if 'radius' in node:
                node['radius'] = str(node['radius'])

        # xml file for nodes; contains nodes for the boundary points with
        # respect to the x and y axes
        x = makexml('nodes', 'http://sumo.dlr.de/xsd/nodes_file.xsd')
        for node_attributes in nodes:
            x.append(E('node', **node_attributes))
        printxml(x, self.net_path + self.nodfn)

        # modify the length, shape, numLanes, and speed values
        for edge in edges:
            edge['length'] = str(edge['length'])
            if 'priority' in edge:
                edge['priority'] = str(edge['priority'])
            if 'shape' in edge:
                if not isinstance(edge['shape'], str):
                    edge['shape'] = ' '.join('%.2f,%.2f' % (x, y)
                                             for x, y in edge['shape'])
            if 'numLanes' in edge:
                edge['numLanes'] = str(edge['numLanes'])
            if 'speed' in edge:
                edge['speed'] = str(edge['speed'])

        # xml file for edges
        x = makexml('edges', 'http://sumo.dlr.de/xsd/edges_file.xsd')
        for edge_attributes in edges:
            x.append(E('edge', attrib=edge_attributes))
        printxml(x, self.net_path + self.edgfn)

        # xml file for types: contains the the number of lanes and the speed
        # limit for the lanes
        if types is not None:
            # modify the numLanes and speed values
            for typ in types:
                if 'numLanes' in typ:
                    typ['numLanes'] = str(typ['numLanes'])
                if 'speed' in typ:
                    typ['speed'] = str(typ['speed'])

            x = makexml('types', 'http://sumo.dlr.de/xsd/types_file.xsd')
            for type_attributes in types:
                x.append(E('type', **type_attributes))
            printxml(x, self.net_path + self.typfn)

        # xml for connections: specifies which lanes connect to which in the
        # edges
        if connections is not None:
            # modify the fromLane and toLane values
            for connection in connections:
                if 'fromLane' in connection:
                    connection['fromLane'] = str(connection['fromLane'])
                if 'toLane' in connection:
                    connection['toLane'] = str(connection['toLane'])

            x = makexml('connections',
                        'http://sumo.dlr.de/xsd/connections_file.xsd')
            for connection_attributes in connections:
                if 'signal_group' in connection_attributes:
                    del connection_attributes['signal_group']
                x.append(E('connection', **connection_attributes))
            printxml(x, self.net_path + self.confn)

        # xml file for configuration, which specifies:
        # - the location of all files of interest for sumo
        # - output net file
        # - processing parameters for no internal links and no turnarounds
        x = makexml('configuration',
                    'http://sumo.dlr.de/xsd/netconvertConfiguration.xsd')
        t = E('input')
        t.append(E('node-files', value=self.nodfn))
        t.append(E('edge-files', value=self.edgfn))
        if types is not None:
            t.append(E('type-files', value=self.typfn))
        if connections is not None:
            t.append(E('connection-files', value=self.confn))
        x.append(t)
        t = E('output')
        t.append(E('output-file', value=self.netfn))
        x.append(t)
        t = E('processing')
        t.append(E('no-internal-links', value='false'))
        t.append(E('no-turnarounds', value='true'))
        x.append(t)
        printxml(x, self.net_path + self.cfgfn)

        subprocess.call(
            [
                'netconvert -c ' + self.net_path + self.cfgfn +
                ' --output-file=' + self.cfg_path + self.netfn +
                ' --no-internal-links="false"'
            ],
            stdout=subprocess.DEVNULL,
            shell=True)

        # collect data from the generated network configuration file
        error = None
        for _ in range(RETRIES_ON_ERROR):
            try:
                edges_dict, conn_dict = self._import_edges_from_net()
                return edges_dict, conn_dict
            except Exception as e:
                print('Error during start: {}'.format(e))
                print('Retrying in {} seconds...'.format(WAIT_ON_ERROR))
                time.sleep(WAIT_ON_ERROR)
        raise error

    def _import_edges_from_net(self):
        parser = etree.XMLParser(recover=True)
        net_path = os.path.join(self.cfg_path, self.netfn)
        tree = ElementTree.parse(net_path, parser=parser)
        root = tree.getroot()

        # Collect information on the available types (if any are available).
        # This may be used when specifying some edge data.
        types_data = dict()

        for typ in root.findall('type'):
            type_id = typ.attrib['id']
            types_data[type_id] = dict()

            if 'speed' in typ.attrib:
                types_data[type_id]['speed'] = float(typ.attrib['speed'])
            else:
                types_data[type_id]['speed'] = None

            if 'numLanes' in typ.attrib:
                types_data[type_id]['numLanes'] = int(typ.attrib['numLanes'])
            else:
                types_data[type_id]['numLanes'] = None

        net_data = dict()
        next_conn_data = dict()  # forward looking connections
        prev_conn_data = dict()  # backward looking connections

        # collect all information on the edges and junctions
        for edge in root.findall('edge'):
            edge_id = edge.attrib['id']

            if edge_id[0] == ":":
                for l in edge.iter("lane"):
                    l.attrib['length'] = str(0)
            # create a new key for this edge
            net_data[edge_id] = dict()

            # check for speed
            if 'speed' in edge:
                net_data[edge_id]['speed'] = float(edge.attrib['speed'])
            else:
                net_data[edge_id]['speed'] = None

            # if the edge has a type parameters, check that type for a
            # speed and parameter if one was not already found
            if 'type' in edge.attrib and edge.attrib['type'] in types_data:
                if net_data[edge_id]['speed'] is None:
                    net_data[edge_id]['speed'] = \
                        float(types_data[edge.attrib['type']]['speed'])

            # collect the length from the lane sub-element in the edge, the
            # number of lanes from the number of lane elements, and if needed,
            # also collect the speed value (assuming it is there)
            net_data[edge_id]['lanes'] = 0
            for i, lane in enumerate(edge):
                net_data[edge_id]['lanes'] += 1
                if i == 0:
                    net_data[edge_id]['length'] = float(lane.attrib['length'])
                    if net_data[edge_id]['speed'] is None \
                            and 'speed' in lane.attrib:
                        net_data[edge_id]['speed'] = float(
                            lane.attrib['speed'])

            # if no speed value is present anywhere, set it to some default
            if net_data[edge_id]['speed'] is None:
                net_data[edge_id]['speed'] = 30

        tree.write(net_path)
        # collect connection data
        for connection in root.findall('connection'):
            from_edge = connection.attrib['from']
            from_lane = int(connection.attrib['fromLane'])

            if from_edge[0] != ":":
                # if the edge is not an internal link, then get the next
                # edge/lane pair from the "via" element
                via = connection.attrib['via'].rsplit('_', 1)
                to_edge = via[0]
                to_lane = int(via[1])
            else:
                to_edge = connection.attrib['to']
                to_lane = int(connection.attrib['toLane'])

            if from_edge not in next_conn_data:
                next_conn_data[from_edge] = dict()

            if from_lane not in next_conn_data[from_edge]:
                next_conn_data[from_edge][from_lane] = list()

            if to_edge not in prev_conn_data:
                prev_conn_data[to_edge] = dict()

            if to_lane not in prev_conn_data[to_edge]:
                prev_conn_data[to_edge][to_lane] = list()

            next_conn_data[from_edge][from_lane].append((to_edge, to_lane))
            prev_conn_data[to_edge][to_lane].append((from_edge, from_lane))

        connection_data = {'next': next_conn_data, 'prev': prev_conn_data}

        return net_data, connection_data

    def generate_cfg(self, net_params, traffic_lights, routes):
        parser = etree.XMLParser(recover=True)
        tree = ElementTree.parse("/home/vamsi/Documents/GitHub/rl-sumo/rlsumo/simulator/sumo_config/cfg/ring_road_20230617-1140031686982203.6779535.add.xml", parser=parser)
        tree.write(self.cfg_path + self.addfn)

        # add = makexml('additional',
        #               'http://sumo.dlr.de/xsd/additional_file.xsd')
        #
        # # add the types of vehicles to the xml file
        # for params in self.network.vehicles.types:
        #     type_params_str = {
        #         key: str(params['type_params'][key])
        #         for key in params['type_params']
        #     }
        #     add.append(E('vType', id=params['veh_id'], **type_params_str))

        # add (optionally) the traffic light properties to the .add.xml file
        num_traffic_lights = len(list(traffic_lights.get_properties().keys()))
        if num_traffic_lights > 0:
            if traffic_lights.baseline:
                tl_params = traffic_lights.actuated_default()
                tl_type = str(tl_params['tl_type'])
                program_id = str(tl_params['program_id'])
                phases = tl_params['phases']
                max_gap = str(tl_params['max_gap'])
                detector_gap = str(tl_params['detector_gap'])
                show_detector = tl_params['show_detectors']

                detectors = {'key': 'detector-gap', 'value': detector_gap}
                gap = {'key': 'max-gap', 'value': max_gap}

                if show_detector:
                    show_detector = {'key': 'show-detectors', 'value': 'true'}
                else:
                    show_detector = {'key': 'show-detectors', 'value': 'false'}

                nodes = self._inner_nodes  # nodes where there's traffic lights
                tll = []
                for node in nodes:
                    tll.append({
                        'id': node['id'],
                        'type': tl_type,
                        'programID': program_id
                    })

                for elem in tll:
                    e = E('tlLogic', **elem)
                    e.append(E('param', **show_detector))
                    e.append(E('param', **gap))
                    e.append(E('param', **detectors))
                    for phase in phases:
                        e.append(E('phase', **phase))
                    # add.append(e)

            else:
                tl_properties = traffic_lights.get_properties()
                for node in tl_properties.values():
                    # At this point, we assume that traffic lights are properly
                    # formed. If there are no phases for a static traffic
                    # light, ignore and use default
                    if node['type'] == 'static' and not node.get('phases'):
                        continue

                    elem = {
                        'id': str(node['id']),
                        'type': str(node['type']),
                        'programID': str(node['programID'])
                    }
                    if node.get('offset'):
                        elem['offset'] = str(node.get('offset'))

                    e = E('tlLogic', **elem)
                    for key, value in node.items():
                        if key == 'phases':
                            for phase in node.get('phases'):
                                e.append(E('phase', **phase))
                        else:
                            e.append(
                                E('param', **{
                                    'key': key,
                                    'value': str(value)
                                }))

                    # add.append(e)

        # printxml(add, self.cfg_path + self.addfn)

        # this is the data that we will pass to the *.gui.cfg file
        gui = E('viewsettings')
        gui.append(E('scheme', name='real world'))
        gui.append(
            E('background',
              backgroundColor='100,100,100',
              showGrid='0',
              gridXSize='100.00',
              gridYSize='100.00'))
        printxml(gui, self.cfg_path + self.guifn)

        # this is the data that we will pass to the *.rou.xml file
        routes_data = makexml('routes',
                              'http://sumo.dlr.de/xsd/routes_file.xsd')

        # add the routes to the .add.xml file
        for route_id in routes.keys():
            # in this case, we only have one route, convert into into a
            # list of routes with one element
            if isinstance(routes[route_id][0], str):
                routes[route_id] = [(routes[route_id], 1)]

            # add each route incrementally, and add a second term to denote
            # the route number of the given route at the given edge
            for i in range(len(routes[route_id])):
                r, _ = routes[route_id][i]
                routes_data.append(E(
                    'route',
                    id='route{}_{}'.format(route_id, i),
                    edges=' '.join(r)
                ))

        # add the inflows from various edges to the xml file
        if self.network.net_params.inflows is not None:
            total_inflows = self.network.net_params.inflows.get()
            for inflow in total_inflows:
                # do not want to affect the original values
                sumo_inflow = deepcopy(inflow)

                # convert any non-string element in the inflow dict to a string
                for key in sumo_inflow:
                    if not isinstance(sumo_inflow[key], str):
                        sumo_inflow[key] = repr(sumo_inflow[key])

                edge = sumo_inflow['edge']
                del sumo_inflow['edge']

                if 'route' not in sumo_inflow:
                    # distribute the inflow rates across all routes from a
                    # given edge w.r.t. the provided fractions for each route
                    for i, (_, ft) in enumerate(routes[edge]):
                        sumo_inflow['name'] += str(i)
                        sumo_inflow['route'] = 'route{}_{}'.format(edge, i)

                        for key in ['vehsPerHour', 'probability', 'period']:
                            if key in sumo_inflow:
                                sumo_inflow[key] = str(float(inflow[key]) * ft)

                        if 'number' in sumo_inflow:
                            sumo_inflow['number'] = str(
                                int(float(inflow['number']) * ft))

                        routes_data.append(_flow(**sumo_inflow))
                else:
                    routes_data.append(_flow(**sumo_inflow))

        printxml(routes_data, self.cfg_path + self.roufn)

        # this is the data that we will pass to the *.sumo.cfg file
        cfg = makexml('configuration',
                      'http://sumo.dlr.de/xsd/sumoConfiguration.xsd')

        cfg.append(
            _inputs(
                net=self.netfn,
                add=self.addfn,
                rou=self.roufn,
                gui=self.guifn))
        t = E('time')
        t.append(E('begin', value=repr(0)))
        cfg.append(t)

        printxml(cfg, self.cfg_path + self.sumfn)
        return self.sumfn

    def close(self):
        """Close the network class.

        Deletes the xml files that were created by the network class. This
        is to prevent them from building up in the debug folder. Note that in
        the case of import .net.xml files we do not want to delete them.
        """
        # Those files are being created even if self.network.net_params.template is a path to .net.xml file
        files = []
        if self.cfg_path != None and self.guifn != None:
            files = [self.cfg_path + self.guifn,
                 self.cfg_path + self.addfn,
                 self.cfg_path + self.roufn,
                 self.cfg_path + self.sumfn]

        if self.network != None:

            if self.network.net_params.template is None:
                files += [self.net_path + self.nodfn,
                      self.net_path + self.edgfn,
                      self.net_path + self.cfgfn,
                      self.net_path + self.confn,
                      self.net_path + self.typfn,
                      self.cfg_path + self.netfn]

        for file in files:
            try:
                os.remove(file)
            except (FileNotFoundError, OSError):
                # the files were never created
                # the connection file is not always created
                # neither is the type file
                continue

    def get_edge(self, x):
        """See parent class."""
        for (edge, start_pos) in reversed(self.total_edgestarts):
            if x >= start_pos:
                return edge, x - start_pos

    def get_x(self, edge, position):
        """See parent class."""
        # if there was a collision which caused the vehicle to disappear,
        # return an x value of -1001
        if len(edge) == 0:
            return -1001

        if edge[0] == ':':
            try:
                return self.internal_edgestarts_dict[edge] + position
            except KeyError:
                # in case several internal links are being generalized for
                # by a single element (for backwards compatibility)
                edge_name = edge.rsplit('_', 1)[0]
                return self.total_edgestarts_dict.get(edge_name, -1001)
        else:
            return self.total_edgestarts_dict[edge] + position

    def edge_length(self, edge_id):
        """See parent class."""
        try:
            return self._edges[edge_id]['length']
        except KeyError:
            print('Error in edge length with key', edge_id)
            return -1001

    def length(self):
        """See parent class."""
        return self.__length

    def non_internal_length(self):
        """See parent class."""
        return self.__non_internal_length

    def speed_limit(self, edge_id):
        """See parent class."""
        try:
            return self._edges[edge_id]['speed']
        except KeyError:
            print('Error in speed limit with key', edge_id)
            return -1001

    def num_lanes(self, edge_id):
        """See parent class."""
        try:
            return self._edges[edge_id]['lanes']
        except KeyError:
            print('Error in num lanes with key', edge_id)
            return -1001

    def max_speed(self):
        """See parent class."""
        return self.__max_speed

    def get_edge_list(self):
        """See parent class."""
        return self._edge_list

    def get_junction_list(self):
        """See parent class."""
        return self._junction_list

    def next_edge(self, edge, lane):
        """See parent class."""
        try:
            return self._connections['next'][edge][lane]
        except KeyError:
            return []

    def prev_edge(self, edge, lane):
        """See parent class."""
        try:
            return self._connections['prev'][edge][lane]
        except KeyError:
            return []