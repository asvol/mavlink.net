﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace Asv.Mavlink {
    using System;
    
    
    /// <summary>
    ///   A strongly-typed resource class, for looking up localized strings, etc.
    /// </summary>
    // This class was auto-generated by the StronglyTypedResourceBuilder
    // class via a tool like ResGen or Visual Studio.
    // To add or remove a member, edit your .ResX file then rerun ResGen
    // with the /str option, or rebuild your VS project.
    [global::System.CodeDom.Compiler.GeneratedCodeAttribute("System.Resources.Tools.StronglyTypedResourceBuilder", "4.0.0.0")]
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
    [global::System.Runtime.CompilerServices.CompilerGeneratedAttribute()]
    internal class RS {
        
        private static global::System.Resources.ResourceManager resourceMan;
        
        private static global::System.Globalization.CultureInfo resourceCulture;
        
        [global::System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal RS() {
        }
        
        /// <summary>
        ///   Returns the cached ResourceManager instance used by this class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Resources.ResourceManager ResourceManager {
            get {
                if (object.ReferenceEquals(resourceMan, null)) {
                    global::System.Resources.ResourceManager temp = new global::System.Resources.ResourceManager("Asv.Mavlink.RS", typeof(RS).Assembly);
                    resourceMan = temp;
                }
                return resourceMan;
            }
        }
        
        /// <summary>
        ///   Overrides the current thread's CurrentUICulture property for all
        ///   resource lookups using this strongly typed resource class.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Globalization.CultureInfo Culture {
            get {
                return resourceCulture;
            }
            set {
                resourceCulture = value;
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Error for deserialize mavlink V2 package with messageID {0}: {1}.
        /// </summary>
        internal static string DecoderV2_TryDecodePacket_Error_for_deserialize_mavlink_V2 {
            get {
                return ResourceManager.GetString("DecoderV2_TryDecodePacket_Error_for_deserialize_mavlink_V2", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Mavlink packet version v2 with MessageId=&apos;{0}&apos; not found in decoder.
        /// </summary>
        internal static string MessageIdNotFoundException_MessageIdNotFoundException {
            get {
                return ResourceManager.GetString("MessageIdNotFoundException_MessageIdNotFoundException", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Error message id type. Want {0}, Got {1}.
        /// </summary>
        internal static string PacketV2_Deserialize_Error_message_id_type {
            get {
                return ResourceManager.GetString("PacketV2_Deserialize_Error_message_id_type", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Bad X25Crc: want {0}, got {1}.
        /// </summary>
        internal static string PacketV2Helper_VerifyCrc_Bad_X25Crc {
            get {
                return ResourceManager.GetString("PacketV2Helper_VerifyCrc_Bad_X25Crc", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Connection string &apos;{0}&apos; is invalid.
        /// </summary>
        internal static string RemoteStreamFactory_CreateStream_Connection_string_is_invalid {
            get {
                return ResourceManager.GetString("RemoteStreamFactory_CreateStream_Connection_string_is_invalid", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Looks up a localized string similar to Unknown STX value. Need {0}. Get {1}.
        /// </summary>
        internal static string WheelKnownConstant_VerifyStx_Unknown_STX_value {
            get {
                return ResourceManager.GetString("WheelKnownConstant_VerifyStx_Unknown_STX_value", resourceCulture);
            }
        }
    }
}
