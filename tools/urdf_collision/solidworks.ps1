# Offline native SolidWorks checks only. Does not connect to the robot.
[CmdletBinding()]
param(
    [ValidateSet('Inspect', 'Save', 'Check', 'Show', 'SelfTest')]
    [string]$Action = 'Inspect',
    [string]$OutputDirectory,
    [string]$InteropDirectory = 'D:\SW2023\SOLIDWORKS\api\redist',
    [string]$Plan,
    [string]$PoseLabel = 'coupled_interior_full'
)
$ErrorActionPreference = 'Stop'
if (!$OutputDirectory) {
    $OutputDirectory = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) '..\..\build\solidworks_collision_v12'
}
$OutputDirectory = [IO.Path]::GetFullPath($OutputDirectory)
Add-Type -Path (Join-Path $InteropDirectory 'SolidWorks.Interop.sldworks.dll')
Add-Type -ReferencedAssemblies (Join-Path $InteropDirectory 'SolidWorks.Interop.sldworks.dll') -TypeDefinition @'
using System;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;

public class SwPartSnapshot {
    public string Name, Path;
    public double[] Transform, Bounds, Centroid;
    public double Volume;
    public int SolidBodies;
    public bool Fixed;
}
public class SwOverlap {
    public string[] Components;
    public double VolumeMm3;
    public bool Possible;
}
public class YlampSolidWorks {
    private ISldWorks sw;
    private IModelDoc2 doc;
    private IAssemblyDoc assembly;
    public string Revision { get { return sw.RevisionNumber(); } }
    public string Title { get { return doc.GetTitle(); } }
    public string Path { get { return doc.GetPathName(); } }

    public YlampSolidWorks() {
        // The installed COM server returns the current session. Do not close user documents.
        sw = (ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application.31", true));
        sw.Visible = true;
        sw.UserControl = true;
        doc = sw.IActiveDoc2;
        if (doc == null || doc.GetType() != 2)
            throw new InvalidOperationException("Activate the imported Ylamp assembly first.");
        assembly = (IAssemblyDoc)doc;
    }
    public SwPartSnapshot[] Inspect() {
        var result = new List<SwPartSnapshot>();
        var components = (object[])assembly.GetComponents(false);
        if (components == null) throw new InvalidOperationException("No resolved assembly components.");
        foreach (IComponent2 c in components) {
            object bodyInfo;
            var bodies = (object[])c.GetBodies3(0, out bodyInfo);
            var info = new SwPartSnapshot { Name=c.Name2, Path=c.GetPathName(),
                Transform=(double[])c.Transform2.ArrayData, Fixed=c.IsFixed(),
                Bounds=new double[] {double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity,
                    double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity},
                Centroid=new double[3] };
            if (bodies != null) foreach (IBody2 body in bodies) {
                double[] props = (double[])body.GetMassProperties(1.0);
                if (props == null || props.Length < 6 || props[3] <= 0)
                    throw new InvalidOperationException("Missing solid mass properties: "+c.Name2);
                double[] box = (double[])body.GetBodyBox();
                for (int k=0;k<3;k++) {
                    info.Bounds[k]=Math.Min(info.Bounds[k],box[k]);
                    info.Bounds[k+3]=Math.Max(info.Bounds[k+3],box[k+3]);
                    info.Centroid[k]+=props[k]*props[3];
                }
                info.Volume+=props[3];
                info.SolidBodies++;
            }
            if (info.Volume > 0) for(int k=0;k<3;k++) info.Centroid[k]/=info.Volume;
            result.Add(info);
        }
        return result.ToArray();
    }
    public void Save(string file) {
        int errors=0, warnings=0;
        if (!doc.Extension.SaveAs(file,0,5,null,ref errors,ref warnings) || errors!=0)
            throw new InvalidOperationException("Save failed: errors="+errors+" warnings="+warnings);
        Console.WriteLine("Saved "+file+"; warnings="+warnings);
    }
    private Dictionary<string,IComponent2> PoseComponents(string[] names, double[][] matrices) {
        if(names.Length!=24 || matrices.Length!=24 || new HashSet<string>(names).Count!=24)
            throw new ArgumentException("A pose must contain all 24 unique instances.");
        var lookup=new Dictionary<string,IComponent2>();
        foreach(IComponent2 c in (object[])assembly.GetComponents(false)) lookup.Add(c.Name2,c);
        if(lookup.Count!=24) throw new InvalidOperationException("Unexpected assembly component count.");
        for(int i=0;i<24;i++) {
            if(!lookup.ContainsKey(names[i]) || lookup[names[i]].IsFixed() || matrices[i].Length!=16)
                throw new ArgumentException("Unexpected component or transform: "+names[i]);
            foreach(double v in matrices[i]) if(Double.IsNaN(v) || Double.IsInfinity(v))
                throw new ArgumentException("Non-finite transform: "+names[i]);
        }
        return lookup;
    }
    private void VerifyPose(Dictionary<string,IComponent2> lookup, string[] names,
                            double[][] matrices, string stage) {
        for(int i=0;i<names.Length;i++) {
            var actual=(double[])lookup[names[i]].Transform2.ArrayData;
            var displayed=(double[])lookup[names[i]].GetTotalTransform(true).ArrayData;
            for(int k=0;k<16;k++)
                if(Double.IsNaN(actual[k]) || Double.IsNaN(displayed[k])
                    || Math.Abs(actual[k]-matrices[i][k])>1e-9
                    || Math.Abs(displayed[k]-matrices[i][k])>1e-9)
                    throw new InvalidOperationException(stage+" transform mismatch: "+names[i]);
        }
    }
    private void ApplyPose(Dictionary<string,IComponent2> lookup, string[] names, double[][] matrices) {
        var math=(IMathUtility)sw.GetMathUtility();
        for(int i=0;i<names.Length;i++)
            lookup[names[i]].Transform2=(MathTransform)math.CreateTransform(matrices[i]);
        doc.EditRebuild3();
        assembly.UpdateBox();
        VerifyPose(lookup,names,matrices,"Applied pose");
    }
    private double[][] CapturePose(Dictionary<string,IComponent2> lookup, string[] names) {
        var matrices=new double[names.Length][];
        for(int i=0;i<names.Length;i++)
            matrices[i]=(double[])((double[])lookup[names[i]].Transform2.ArrayData).Clone();
        VerifyPose(lookup,names,matrices,"Initial pose");
        return matrices;
    }
    public void ShowAndSavePose(string file, string[] names, double[][] absoluteTransforms) {
        PoseComponents(names,absoluteTransforms);
        int errors=0, warnings=0;
        // Copy-and-open before moving anything: the baseline file stays at CAD zero.
        if(!doc.Extension.SaveAs(file,0,513,null,ref errors,ref warnings) || errors!=0)
            throw new InvalidOperationException("Cannot copy scene: "+errors);
        doc=sw.IActiveDoc2;
        if(!String.Equals(doc.GetPathName(),file,StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("Copy-and-open did not activate the requested scene.");
        assembly=(IAssemblyDoc)doc;
        var lookup=PoseComponents(names,absoluteTransforms);
        var originals=CapturePose(lookup,names);
        var view=(IModelView)doc.IActiveView;
        bool refresh=view.EnableGraphicsUpdate, saved=false;
        try {
            view.EnableGraphicsUpdate=false;
            ApplyPose(lookup,names,absoluteTransforms);
            doc.ShowNamedView2("*Isometric",7);
            doc.ViewZoomtofit2();
            if(!doc.Save3(1,ref errors,ref warnings) || errors!=0)
                throw new InvalidOperationException("Scene save failed: "+errors);
            saved=true;
        } finally {
            try { if(!saved) ApplyPose(lookup,names,originals); }
            finally { view.EnableGraphicsUpdate=refresh; doc.GraphicsRedraw2(); }
        }
        Console.WriteLine("Verified all 24 transforms and saved scene: "+file);
    }
    public SwOverlap[] Check(string[] names, double[][] absoluteTransforms) {
        var lookup=PoseComponents(names,absoluteTransforms);
        var originals=CapturePose(lookup,names);
        var view=(IModelView)doc.IActiveView;
        bool refresh=view.EnableGraphicsUpdate;
        var manager=(IInterferenceDetectionMgr)assembly.InterferenceDetectionManager;
        try {
            view.EnableGraphicsUpdate=false;
            doc.ClearSelection2(true);
            // Check the verified physical pose, not the manager's temporary transform preview.
            manager.UseTransform=false;
            ApplyPose(lookup,names,absoluteTransforms);
            manager.TreatCoincidenceAsInterference=false;
            manager.TreatSubAssembliesAsComponents=false;
            manager.IncludeMultibodyPartInterferences=false;
            manager.IgnoreHiddenBodies=false;
            var overlaps=(object[])manager.GetInterferences();
            var result=new List<SwOverlap>();
            if(overlaps!=null) foreach(IInterference hit in overlaps) {
                var pair=new List<string>();
                foreach(IComponent2 c in (object[])hit.Components) {
                    if(Array.IndexOf(names,c.Name2)<0)
                        throw new InvalidOperationException("Interference outside selected scope: "+c.Name2);
                    pair.Add(c.Name2);
                }
                double volume=hit.Volume*1e9;
                if(Double.IsNaN(volume) || Double.IsInfinity(volume) || volume<0)
                    throw new InvalidOperationException("Invalid native interference volume.");
                result.Add(new SwOverlap {Components=pair.ToArray(),VolumeMm3=volume,
                    Possible=hit.IsPossibleInterference});
            }
            VerifyPose(lookup,names,absoluteTransforms,"Checked pose");
            return result.ToArray();
        } finally {
            try {
                manager.Done();
            } finally {
                try {
                    ApplyPose(lookup,names,originals);
                    doc.ClearSelection2(true);
                } finally {
                    view.EnableGraphicsUpdate=refresh;
                    doc.GraphicsRedraw2();
                }
            }
        }
    }
    public static void SelfTest() {
        // Verify the COM transform convention with the actual installed math utility.
        var app=(ISldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application.31",true));
        app.Visible=true;
        app.UserControl=true;
        var math=(IMathUtility)app.GetMathUtility();
        var t=(IMathTransform)math.CreateTransform(new double[] {
            0,1,0, -1,0,0, 0,0,1, 0.1,0.2,0.3, 1,0,0,0 });
        var p=(IMathPoint)math.CreatePoint(new double[]{1,0,0});
        double[] actual=(double[])((IMathPoint)p.MultiplyTransform(t)).ArrayData;
        if(Math.Abs(actual[0]-.1)>1e-12 || Math.Abs(actual[1]-1.2)>1e-12
            || Math.Abs(actual[2]-.3)>1e-12) throw new Exception("Native transform convention mismatch.");
        Console.WriteLine("SELF_TEST PASS: native SolidWorks row-vector rotation and metre translation.");
    }
}
'@

if ($Action -eq 'SelfTest') {
    [YlampSolidWorks]::SelfTest()
    return
}
$session = [YlampSolidWorks]::new()
$snapshot = @($session.Inspect())
$stamp = [DateTimeOffset]::UtcNow.ToString('yyyyMMdd_HHmmss_fff')
[IO.Directory]::CreateDirectory($OutputDirectory) | Out-Null
if ($Action -eq 'Save') {
    $file = Join-Path $OutputDirectory 'ylamp_v12.SLDASM'
    if (Test-Path -LiteralPath $file) { throw "Will not overwrite $file" }
    if ($snapshot.Count -ne 24 -or @($snapshot | Where-Object {
        !$_.Path.StartsWith($OutputDirectory + '\', [StringComparison]::OrdinalIgnoreCase)
    }).Count) { throw 'Only the 24-component independent STEP import may be saved.' }
    $session.Save($file)
} elseif ($Action -eq 'Inspect') {
    $report = [ordered]@{
        utc = [DateTimeOffset]::UtcNow.ToString('o')
        revision = $session.Revision
        title = $session.Title
        assemblyPath = $session.Path
        units = 'metres, cubic metres; native component-local solids'
        components = $snapshot
    }
    $file = Join-Path $OutputDirectory "inspection_$stamp.json"
    [IO.File]::WriteAllText($file, ($report | ConvertTo-Json -Depth 12))
    $snapshot | Select-Object Name,SolidBodies,Volume,Fixed | Format-Table -AutoSize
    Write-Output "Inspection: $file"
} else {
    if (!$Plan) { throw 'Pass -Plan with verified native component names and absolute transforms.' }
    $nativePath = [IO.Path]::GetFullPath($session.Path)
    if (!$nativePath.StartsWith($OutputDirectory + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Check only accepts an active assembly in the independent output directory.'
    }
    if ($snapshot.Count -ne 24 -or @($snapshot | Where-Object {
        !$_.Path.StartsWith($OutputDirectory + '\', [StringComparison]::OrdinalIgnoreCase)
    }).Count) { throw 'Only the complete independent assembly and its copied parts may be moved.' }
    $planData = Get-Content -LiteralPath $Plan -Raw -Encoding utf8 | ConvertFrom-Json
    $mappingNames = @($planData.componentMapping | Select-Object -ExpandProperty name -Unique)
    if (!$planData.poses.Count -or $planData.componentMapping.Count -ne 24 -or $mappingNames.Count -ne 24 -or
        @($snapshot | Where-Object { $_.Name -notin $mappingNames }).Count -or
        @($planData.componentMapping | Where-Object { [string]::IsNullOrWhiteSpace($_.link) }).Count -or
        @($planData.componentMapping | Select-Object -ExpandProperty link -Unique).Count -ne 6) {
        throw 'Plan must map all 24 native instances to six rigid links and contain test poses.'
    }
    if ($Action -eq 'Show') {
        if ($PoseLabel -notmatch '^[A-Za-z0-9_-]+$') { throw 'Invalid pose label.' }
        $pose = @($planData.displayPoses | Where-Object { $_.label -eq $PoseLabel })
        if ($pose.Count -ne 1) { throw 'Select exactly one existing display pose.' }
        $pose = $pose[0]
        [string[]]$names = @($pose.components | ForEach-Object { $_.name })
        [double[][]]$transforms = @($pose.components | ForEach-Object { ,([double[]]$_.transform) })
        $file = Join-Path $OutputDirectory "pose_$PoseLabel.SLDASM"
        if (Test-Path -LiteralPath $file) { throw "Will not overwrite $file" }
        $session.ShowAndSavePose($file, $names, $transforms)
        return
    }
    $results = @()
    $runTimer = [Diagnostics.Stopwatch]::StartNew()
    $planHash = (Get-FileHash -LiteralPath $Plan -Algorithm SHA256).Hash.ToLowerInvariant()
    $traceFile = Join-Path $OutputDirectory "interferences_$stamp.jsonl"
    $traceStream = [IO.File]::Open($traceFile, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    $trace = [IO.StreamWriter]::new($traceStream, [Text.UTF8Encoding]::new($false))
    $trace.AutoFlush = $true
    try {
        $trace.WriteLine((@{ kind='run'; utc=[DateTimeOffset]::UtcNow.ToString('o')
            planSha256=$planHash; plannedPoses=$planData.poses.Count; assemblyPath=$session.Path
        } | ConvertTo-Json -Compress))
        foreach ($pose in $planData.poses) {
            $poseTimer = [Diagnostics.Stopwatch]::StartNew()
            [string[]]$names = @($pose.components | ForEach-Object { $_.name })
            [double[][]]$transforms = @($pose.components | ForEach-Object { ,([double[]]$_.transform) })
            if ($names.Count -ne 24) { throw 'Every pose must transform all 24 instances, including hardware.' }
            $rawHits = @($session.Check($names, $transforms))
            $crossLinkHits = @($rawHits | Where-Object {
                $pair = $_.Components
                @($planData.componentMapping | Where-Object { $_.name -in $pair } |
                    Select-Object -ExpandProperty link -Unique).Count -gt 1
            })
            $confirmedHits = @($crossLinkHits | Where-Object { !$_.Possible -and $_.VolumeMm3 -gt 0 })
            $possibleHits = @($crossLinkHits | Where-Object { $_.Possible -or $_.VolumeMm3 -le 0 })
            $scopedHits = @($crossLinkHits | Where-Object {
                $pair = $_.Components
                $outside = @($pair | Where-Object { $_ -notin $pose.crossLinkScope })
                !$outside.Count
            })
            $result = [ordered]@{ label=$pose.label; cadDegrees=$pose.cadDegrees
                overlaps=$scopedHits; crossLinkOverlaps=$crossLinkHits; rawWholeAssemblyOverlaps=$rawHits
                confirmedCrossLinkOverlaps=$confirmedHits; possibleCrossLinkOverlaps=$possibleHits
                reportingScope=$pose.crossLinkScope; restoredAndVerified=$true
                elapsedSeconds=$poseTimer.Elapsed.TotalSeconds }
            $results += $result
            $trace.WriteLine((@{ kind='pose'; result=$result } | ConvertTo-Json -Compress -Depth 20))
            Write-Output "$($results.Count)/$($planData.poses.Count) $($pose.label): $($confirmedHits.Count) confirmed / $($possibleHits.Count) possible cross-link interferences; $($rawHits.Count) whole-assembly records; restored 24/24"
        }
    } finally {
        $trace.Dispose()
    }
    $file = Join-Path $OutputDirectory "interferences_$stamp.json"
    [IO.File]::WriteAllText($file, ([ordered]@{
        utc=[DateTimeOffset]::UtcNow.ToString('o'); revision=$session.Revision
        assemblyPath=$session.Path; planPath=[IO.Path]::GetFullPath($Plan); results=$results
        planSha256=$planHash; completed=$true; elapsedSeconds=$runTimer.Elapsed.TotalSeconds
        method='Verified component Transform2 poses; interference manager UseTransform=false; explicit verified restoration.'
        note='Discrete exact-solid checks, not continuous motion or a safety certificate.'
    } | ConvertTo-Json -Depth 20))
    Write-Output "Results: $file"
}
