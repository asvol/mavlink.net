namespace Asv.Mavlink
{
    public static class WellKnownDiag
    {
        #region DIAG

        public const string Diag = "DIAG";
        public const string DiagStringsValueName = "SVAL";
        public const string DiagDigitValueName = "DVAL";
        public const string DiagSettingsValueName = "CONF";
        public const string DiagSettingsSetMethodName = "SET";

        #endregion
    }

    public class Void
    {
        public static Void Default = new Void();
    }
}
