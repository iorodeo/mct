import json
import json_tools

def set_dict(db,name,data_dict):
    """
    Store dictionary, data_dict,  in redis database using json
    """
    data_json = json.dumps(data_dict)
    db.set(name,data_json)

def get_dict(db,name):
    """
    Retrieve python dictionary stored in redis database as json string.
    """
    data_json = db.get(name)
    data_dict = json.loads(data_json,object_hook=json_tools.decode_dict)
    return data_dict

def set_str(db,name,data_str):
    """
    Sets a string in the redis database.
    """
    db.set(name,data_str)

def get_str(db,name):
    """
    Get a string from the redis database.
    """
    data_unicode = db.get(name)
    return str(data_unicode)

