MATCH (s:Point{id:4065358592})
MATCH (e:Point{id:8961141085})
CALL londonSafeTravel.route(s, e, "crossTimeMotorVehicle", 70.0)
YIELD index, node, time
RETURN index, node, time
  ORDER BY index DESCENDING
