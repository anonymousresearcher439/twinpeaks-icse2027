import unittest


class TestLlaPosition(unittest.TestCase):
    def test_to_amsl(self):
        from src.dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition

        lla = LlaPosition(
            latitude=39.66,
            longitude=-104.98,
            altitude=1234.2,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        )

        lla_asml = lla.to_amsl()

        self.assertAlmostEqual(lla_asml.altitude, 1251.104, 3)
        self.assertEqual(lla_asml.datum_ref, DATUM_REFERENCE.AMSL)


    def test_to_amsl_already_amsl(self):
        from src.dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition

        lla = LlaPosition(
            latitude=39.66,
            longitude=-104.98,
            altitude=907.124,
            datum_ref=DATUM_REFERENCE.AMSL
        )

        lla = lla.to_amsl()

        self.assertEqual(lla.altitude, 907.124)
        self.assertEqual(lla.datum_ref, DATUM_REFERENCE.AMSL)


    def test_to_wgs84_ellipsoid(self):
        from src.dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition

        lla = LlaPosition(
            latitude=39.66,
            longitude=-104.98,
            altitude=706.598,
            datum_ref=DATUM_REFERENCE.AMSL
        )

        lla_wgs84 = lla.to_wgs84_ellipsoid()

        self.assertAlmostEqual(lla_wgs84.altitude, 689.694, 3)
        self.assertEqual(lla_wgs84.datum_ref, DATUM_REFERENCE.ELLIPSOID_WGS84)


    def test_to_wgs84_ellipsoid_already_wgs84_ellipsoid(self):
        from src.dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition

        lla = LlaPosition(
            latitude=39.66,
            longitude=-104.98,
            altitude=256.325,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        )

        lla = lla.to_wgs84_ellipsoid()

        self.assertAlmostEqual(lla.altitude, 256.325, 3)
        self.assertEqual(lla.datum_ref, DATUM_REFERENCE.ELLIPSOID_WGS84)

