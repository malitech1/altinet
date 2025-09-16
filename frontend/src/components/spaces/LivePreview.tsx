import React from "react";

interface Props {
  url?: string | null;
}

const LivePreview: React.FC<Props> = ({ url }) => {
  return (
    <div className="flex h-64 w-full items-center justify-center overflow-hidden rounded border border-slate-800 bg-slate-900">
      {url ? (
        <video className="h-full w-full object-cover" src={url} controls autoPlay muted loop />
      ) : (
        <video
          className="h-full w-full object-cover"
          src="https://storage.googleapis.com/coverr-main/mp4/Mt_Baker.mp4"
          controls
          autoPlay
          muted
          loop
        />
      )}
    </div>
  );
};

export default LivePreview;
