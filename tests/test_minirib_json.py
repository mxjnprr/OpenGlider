import json
import unittest

from openglider import jsonify
from openglider.glider.rib import MiniRib


class TestMiniRibsSerialization(unittest.TestCase):
    def test_jsonify_minirib(self):
        mr = MiniRib(
            yvalue=0.5,
            intrados_start=0.8,
            extrados_start=0.75,
            name="test_mr"
        )
        
        # Test serialization using openglider.jsonify
        try:
            json_str = jsonify.dumps(mr, add_meta=False)
            print(f"Serialized JSON: {json_str}")
        except Exception as e:
            self.fail(f"jsonify.dumps failed: {e}")
            
        # Verify content
        data = json.loads(json_str)
        self.assertEqual(data["_type"], "MiniRib")
        self.assertEqual(data["data"]["yvalue"], 0.5)
        self.assertEqual(data["data"]["intrados_start"], 0.8)
        self.assertEqual(data["data"]["extrados_start"], 0.75)
        self.assertEqual(data["data"]["name"], "test_mr")

if __name__ == "__main__":
    unittest.main()
